"""Explicit socket capabilities gate keyboard activation and report owner loss."""

import fcntl
import os
import socket

import pytest

from nav_bringup.navrun.keyboard_channel import KeyboardChannel, KeyboardOwnerLost


def test_ready_and_go_handshake_keeps_originals_owned():
    """The child announces its guard result and waits for the supervisor's permit."""
    parent, child = socket.socketpair()
    original_flags = fcntl.fcntl(child, fcntl.F_GETFL)
    try:
        with KeyboardChannel(child.fileno()) as channel:
            channel.ready()
            assert parent.recv(1) == b'R'
            assert not channel.permitted()
            parent.sendall(b'G')
            assert channel.permitted()
            channel.check_owner()
        assert child.fileno() >= 3
        assert fcntl.fcntl(child, fcntl.F_GETFL) == original_flags
    finally:
        parent.close()
        child.close()


@pytest.mark.parametrize('before_go', [False, True])
def test_owner_disconnect_is_never_empty_success(before_go):
    """EOF is a stop request both while waiting and while reading keys."""
    parent, child = socket.socketpair()
    try:
        with KeyboardChannel(child.fileno()) as channel:
            if not before_go:
                parent.sendall(b'G')
                assert channel.permitted()
            parent.close()
            with pytest.raises(KeyboardOwnerLost):
                channel.permitted() if before_go else channel.check_owner()
    finally:
        parent.close()
        child.close()


def test_unexpected_control_bytes_fail_closed():
    """The channel accepts one activation byte, not arbitrary control commands."""
    parent, child = socket.socketpair()
    try:
        with KeyboardChannel(child.fileno()) as channel:
            parent.sendall(b'X')
            with pytest.raises(KeyboardOwnerLost):
                channel.permitted()
    finally:
        parent.close()
        child.close()


def test_nonsocket_descriptor_rejected_and_original_preserved(tmp_path):
    """A copied PID file or regular descriptor is not an owner channel."""
    with (tmp_path / 'data').open('wb') as source:
        with pytest.raises((OSError, ValueError)):
            with KeyboardChannel(source.fileno()):
                pytest.fail('not a socket')
        os.fstat(source.fileno())
