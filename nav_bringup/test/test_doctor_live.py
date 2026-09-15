"""Tests for the live doctor checks, with canned ros2 command output."""

import json

from nav_bringup.doctor import checks
from nav_bringup.doctor import live
from nav_bringup.navrun import cli

LOCAL_GID = '01.0f.da.f0.b1.37.b6.ff.00.00.00.00.00.00.05.03.00.00.00.00.00.00.00.00'
REMOTE_GID = '01.0f.12.34.aa.bb.cc.dd.00.00.00.00.00.00.01.03.00.00.00.00.00.00.00.00'
LISTENER_GID = '01.0f.da.f0.99.99.99.99.00.00.00.00.00.00.02.04.00.00.00.00.00.00.00.00'


def _endpoint_block(name, namespace, endpoint_type, gid):
    return (
        f'Node name: {name}\n'
        f'Node namespace: {namespace}\n'
        'Topic type: std_msgs/msg/String\n'
        f'Endpoint type: {endpoint_type}\n'
        f'GID: {gid}\n'
        'QoS profile:\n'
        '  Reliability: RELIABLE\n'
        '  Durability: TRANSIENT_LOCAL\n'
        '\n'
    )


TWO_HOST_INFO = (
    'Type: std_msgs/msg/String\n\nPublisher count: 2\n\n'
    + _endpoint_block('_ros2cli_79793', '/', 'PUBLISHER', LOCAL_GID)
    + _endpoint_block('teleop', '/tools', 'PUBLISHER', REMOTE_GID)
    + 'Subscription count: 1\n\n'
    + _endpoint_block('listener', '/', 'SUBSCRIPTION', LISTENER_GID)
)
ONE_HOST_INFO = (
    'Type: std_msgs/msg/String\n\nPublisher count: 1\n\n'
    + _endpoint_block('driver', '/', 'PUBLISHER', LOCAL_GID)
    + 'Subscription count: 0\n\n'
)


def _info(topic):
    return ('ros2', 'topic', 'info', '--verbose', '--no-daemon', '--spin-time', '3', topic)


def _runner(responses):
    calls = []

    def run(argv):
        calls.append(tuple(argv))
        return responses.get(tuple(argv), (1, ''))

    return run, calls


def _context(responses, **overrides):
    run, calls = _runner(responses)
    return live.LiveContext(runner=run, **overrides), calls


def test_parse_topic_info():
    """Endpoints, their namespaces and GIDs are parsed; QoS details are skipped."""
    topic_type, endpoints = live.parse_topic_info(TWO_HOST_INFO)

    assert topic_type == 'std_msgs/msg/String'
    assert [(e.qualified_node, e.endpoint_type) for e in endpoints] == [
        ('/_ros2cli_79793', 'PUBLISHER'),
        ('/tools/teleop', 'PUBLISHER'),
        ('/listener', 'SUBSCRIPTION'),
    ]
    assert endpoints[1].gid == REMOTE_GID


def test_gid_host_and_daemon_status():
    """Fast DDS host bytes are extracted; daemon status text is recognized."""
    assert live.gid_host(LOCAL_GID) == 'da.f0'
    assert live.gid_host('01.10.aa.bb.cc') is None
    assert live.gid_host('') is None
    assert live.parse_daemon_status('The daemon is not running\n') is False
    assert live.parse_daemon_status('The daemon is running\n') is True
    assert live.parse_daemon_status('something else') is None


def test_caching_runner_runs_each_command_once():
    """Repeated commands replay the first result."""
    run, calls = _runner({('ros2', 'daemon', 'status'): (0, 'The daemon is not running')})
    cached = live.caching_runner(run)

    assert cached(['ros2', 'daemon', 'status']) == cached(['ros2', 'daemon', 'status'])
    assert len(calls) == 1


def test_check_cli_daemon():
    """A daemon that misses nodes warns; a stopped or agreeing daemon passes."""
    status = ('ros2', 'daemon', 'status')
    via_daemon = ('ros2', 'node', 'list')
    direct = ('ros2', 'node', 'list', '--no-daemon', '--spin-time', '3')

    stopped, calls = _context({status: (0, 'The daemon is not running')})
    stale, _ = _context({
        status: (0, 'The daemon is running'),
        via_daemon: (0, ''),
        direct: (0, '/bt_navigator\n/controller_server\n'),
    })
    agreeing, _ = _context({
        status: (0, 'The daemon is running'),
        via_daemon: (0, '/bt_navigator\n'),
        direct: (0, '/bt_navigator\n'),
    })
    broken, _ = _context({})

    assert live.check_cli_daemon(stopped).status is checks.Status.OK
    assert calls == [status]
    result = live.check_cli_daemon(stale)
    assert result.status is checks.Status.WARN
    assert '2/2' in result.message and 'ros2 daemon stop' in result.hint
    assert live.check_cli_daemon(agreeing).status is checks.Status.OK
    assert live.check_cli_daemon(broken).status is checks.Status.SKIP


def test_check_command_publishers():
    """Several publishers on a command topic warn; one publisher or a missing topic pass."""
    duplicated, _ = _context({_info('/cmd'): (0, TWO_HOST_INFO)}, command_topics=('/cmd',))
    single, _ = _context({_info('/cmd'): (0, ONE_HOST_INFO)}, command_topics=('/cmd', '/gone'))
    unset, _ = _context({})

    result = live.check_command_publishers(duplicated)
    assert result.status is checks.Status.WARN
    assert result.message == '/cmd 有 2 个发布者：/_ros2cli_79793，/tools/teleop'
    passed = live.check_command_publishers(single)
    assert passed.status is checks.Status.OK
    assert passed.message == '/cmd：1 个发布者；/gone：不存在'
    assert live.check_command_publishers(unset).status is checks.Status.SKIP


def test_check_foreign_hosts():
    """Publishers from two hosts warn; subscribers are ignored; one host passes."""
    mixed, calls = _context(
        {_info('/map'): (0, TWO_HOST_INFO)}, watched_topics=('/map',), command_topics=('/map',))
    local, _ = _context({_info('/map'): (0, ONE_HOST_INFO)}, watched_topics=('/map',))

    result = live.check_foreign_hosts(mixed)
    assert result.status is checks.Status.WARN
    assert '2 台主机' in result.message and '12.34' in result.message
    assert calls == [_info('/map')]
    assert live.check_foreign_hosts(local).status is checks.Status.OK
    assert live.check_foreign_hosts(_context({})[0]).status is checks.Status.SKIP


def test_check_map_to_odom():
    """A missing transform warns as a hint; no probe or no TF skips."""
    def probe_with(value):
        return _context({}, transform_probe=lambda parent, child: value)[0]

    assert live.check_map_to_odom(_context({})[0]).status is checks.Status.SKIP
    assert live.check_map_to_odom(probe_with(None)).status is checks.Status.SKIP
    assert live.check_map_to_odom(probe_with(False)).status is checks.Status.WARN
    assert live.check_map_to_odom(probe_with(True)).status is checks.Status.OK


def test_run_live_checks_isolates_errors():
    """A live check that raises becomes FAIL."""
    def check_broken(context):
        raise RuntimeError('graph exploded')

    results = live.run_live_checks(_context({})[0], [check_broken, live.check_map_to_odom])

    assert [(r.name, r.status) for r in results] == [
        ('broken', checks.Status.FAIL), ('map_to_odom', checks.Status.SKIP)]


def test_cli_doctor_live_appends_live_results(tmp_path, monkeypatch, capsys):
    """--live adds the live checks with the topics and frames from the command line."""
    monkeypatch.setenv('NAVRUN_REPO', str(tmp_path / 'src'))
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    contexts = []

    def fake_run_live_checks(context):
        contexts.append(context)
        return [checks.CheckResult('map_to_odom', checks.Status.WARN, 'no transform')]

    monkeypatch.setattr(live, 'run_live_checks', fake_run_live_checks)

    code = cli.main([
        'doctor', '--json', '--live', '--cmd-topic', '/cmd', '--watch-topic', '/map',
        '--odom-frame', 'odom_combined',
    ])
    payload = json.loads(capsys.readouterr().out)

    assert code in (0, 1)
    assert payload[-1]['name'] == 'map_to_odom'
    context = contexts[0]
    assert (context.command_topics, context.watched_topics, context.odom_frame) == (
        ('/cmd',), ('/map',), 'odom_combined')
    assert context.transform_probe is not None
