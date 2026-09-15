"""Zero-only shutdown commands are validated and tested without ROS initialization."""

import pytest

from nav_bringup.navrun import zero_command


@pytest.fixture
def transport(monkeypatch):
    """Provide a fake zero transport and virtual monotonic clock."""
    trace = []
    ticks = [0.0]

    class Fake:
        def __init__(self, topics):
            trace.append(('topics', topics))

        def __enter__(self):
            trace.append('open')
            return self

        def __exit__(self, *args):
            trace.append('close')

        def spin(self, seconds):
            ticks[0] += seconds

        def publish(self):
            trace.append('zero')

        def receiver_count(self):
            return 0

    monkeypatch.setattr(zero_command, '_RosZeros', Fake)
    monkeypatch.setattr(zero_command.time, 'monotonic', lambda: ticks[0])
    return trace, ticks


def test_only_five_zero_publications_after_discovery(transport, capsys):
    """No velocity values are accepted; discovery precedes a fixed zero burst."""
    assert zero_command.main(['--topic', '/fixture/cmd', '--topic', '/fixture/cmd']) == 0
    trace, ticks = transport
    assert trace == [('topics', ('/fixture/cmd',)), 'open', *(['zero'] * 5), 'close']
    assert 1.2 <= ticks[0] <= 1.4
    assert 'physical stop is not verified' in capsys.readouterr().out


@pytest.mark.parametrize('argv', [
    [], ['--topic', 'relative'], ['--topic', '/bad//name'],
    ['--topic', '/cmd', '--discovery-sec', '-1'],
    ['--topic', '/cmd', '--discovery-sec', 'nan'],
    ['--topic', '/cmd', '--discovery-sec', '3']])
def test_invalid_input_never_initializes_transport(transport, argv):
    """Malformed topics and unbounded time limits fail before ROS imports."""
    with pytest.raises(SystemExit) as error:
        zero_command.main(argv)
    assert error.value.code == 2
    assert not transport[0]


def test_publish_error_closes_transport_and_reports_failure(transport, monkeypatch):
    """A local publish error is not reported as successful stopping."""
    def fail(self):
        raise RuntimeError('fixture DDS failure')

    monkeypatch.setattr(zero_command._RosZeros, 'publish', fail)
    assert zero_command.main(['--topic', '/fixture/cmd']) == 2
    assert transport[0][-1] == 'close'
