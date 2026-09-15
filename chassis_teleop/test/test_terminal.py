"""Read only test-owned pseudoterminals, never a user's terminal or robot serial port."""

import os
import pty
import termios

import pytest

from chassis_teleop.terminal import TerminalKeyboard, TerminalUnavailable


@pytest.fixture
def terminal_pair():
    """Own a fresh PTY pair and close both endpoints even after test failure."""
    master, slave = pty.openpty()
    try:
        yield master, slave, os.ttyname(slave)
    finally:
        os.close(master)
        os.close(slave)


def test_reads_keys_and_restores_terminal_attributes(terminal_pair):
    """The reader owns a separate CLOEXEC descriptor and restores terminal settings."""
    master, slave, path = terminal_pair
    before = termios.tcgetattr(slave)
    reader = TerminalKeyboard(path)
    with reader:
        assert not os.get_inheritable(reader.fileno())
        assert not termios.tcgetattr(slave)[3] & termios.ICANON
        os.write(master, b'w\x1b[A kq')
        assert [reader.read_key(0.1) for _ in range(5)] == ['w', '\x1b[A', ' ', 'k', 'q']
        assert reader.read_key(0.001) == ''
    assert termios.tcgetattr(slave) == before
    with pytest.raises(RuntimeError):
        reader.fileno()


def test_fragmented_escape_is_not_reinterpreted_as_wasd(terminal_pair):
    """An arrow suffix arriving next poll must not become a bare uppercase A key."""
    master, _, path = terminal_pair
    with TerminalKeyboard(path) as reader:
        os.write(master, b'\x1b[')
        assert reader.read_key(0.01) == ''
        os.write(master, b'A')
        assert reader.read_key(0.1) == '\x1b[A'


def test_long_control_sequence_never_becomes_a_motion_letter(terminal_pair):
    """Unknown CSI sequences remain one non-motion event, including overlong input."""
    master, _, path = terminal_pair
    with TerminalKeyboard(path) as reader:
        os.write(master, b'\x1b[' + b'1;' * 40 + b'Aw')
        assert reader.read_key(0.1) == ''
        assert reader.read_key(0.1) == 'w'


def test_exception_restores_attributes_and_releases_descriptor(terminal_pair):
    """Exceptions during the caller's loop cannot leave cbreak mode behind."""
    _, slave, path = terminal_pair
    before = termios.tcgetattr(slave)
    reader = TerminalKeyboard(path)
    with pytest.raises(ValueError):
        with reader:
            raise ValueError('caller failure')
    assert termios.tcgetattr(slave) == before
    reader.close()


def test_regular_file_and_missing_device_fail_readably(tmp_path):
    """No controlling terminal is an actionable error, not silent success."""
    file_path = tmp_path / 'not_a_terminal'
    file_path.write_text('preserve me')
    for path in (file_path, tmp_path / 'absent'):
        with pytest.raises(TerminalUnavailable, match='TTY'):
            with TerminalKeyboard(str(path)):
                pytest.fail('must reject nonterminals')
    assert file_path.read_text() == 'preserve me'


def test_closed_reader_and_invalid_timeout_fail(terminal_pair):
    """Bad API usage does not leak file descriptors or block indefinitely."""
    _, _, path = terminal_pair
    reader = TerminalKeyboard(path)
    with pytest.raises(RuntimeError):
        reader.read_key(0.1)
    with reader:
        with pytest.raises(ValueError):
            reader.read_key(float('inf'))
        with pytest.raises(RuntimeError):
            reader.__enter__()


def test_setup_failure_restores_terminal_and_closes_fd(terminal_pair, monkeypatch):
    """A partially failed terminal configuration does not leak cbreak settings."""
    from chassis_teleop import terminal

    _, slave, path = terminal_pair
    before = termios.tcgetattr(slave)
    original = terminal.tty.setcbreak
    opened = []

    def fail_after_change(descriptor):
        opened.append(descriptor)
        original(descriptor)
        raise OSError('setup failure')

    monkeypatch.setattr(terminal.tty, 'setcbreak', fail_after_change)
    with pytest.raises(TerminalUnavailable, match='setup failure'):
        with TerminalKeyboard(path):
            pytest.fail('setup must fail')
    assert termios.tcgetattr(slave) == before
    with pytest.raises(OSError):
        os.fstat(opened[0])


def test_read_disconnect_is_not_an_infinite_empty_poll():
    """A hung-up PTY reports a disconnection; the reader still closes its descriptor."""
    master, slave = pty.openpty()
    reader = TerminalKeyboard(os.ttyname(slave))
    try:
        reader.__enter__()
        descriptor = reader.fileno()
        os.close(master)
        master = None
        with pytest.raises(TerminalUnavailable):
            reader.read_key(0.01)
        # A hung-up PTY can also refuse attribute restoration; it must still close.
        try:
            reader.close()
        except TerminalUnavailable:
            pass
        with pytest.raises(OSError):
            os.fstat(descriptor)
    finally:
        reader.close()
        if master is not None:
            os.close(master)
        os.close(slave)
