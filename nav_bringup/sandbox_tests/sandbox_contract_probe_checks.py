"""Actual installed contract CLI against synthetic TF/data and owned process groups."""

import json
import os
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_prefix
import pytest
import yaml

from conftest import require_sandbox
from nav_bringup.navrun.owned_process import OwnedProcess


@pytest.mark.parametrize('variant,expected,code', [
    ('good', 0, None),
    ('silent', 1, 'localization_odom_missing'),
    ('missing_body', 1, 'localization_tf_missing'),
    ('no_driver_sub', 1, 'chassis_cmd_missing'),
    ('wrong_driver_type', 1, 'chassis_cmd_missing'),
    ('waiting', 3, None), ('interrupt', 130, None)])
def test_installed_contract_probe_reads_data_without_starting_providers(
        tmp_path, variant, expected, code):
    """Only synthetic data/TF are published; fixture-owned command input stays passive."""
    require_sandbox()
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({
        'schema_version': 1,
        'frames': {'map': 'probe_world', 'odom': 'probe_odom', 'body': 'probe_body',
                   'nav_base': 'probe_nav'},
        'localization': {'odom_topic': '/contract/loc_odom'},
        'chassis': {'cmd_topic': '/contract/command', 'odom_topic': '/contract/wheel_odom'}}))
    fixture = checker = None
    fixture_log, checker_log = tmp_path / 'fixture.log', tmp_path / 'checker.log'
    try:
        fixture = OwnedProcess([
            sys.executable, str(Path(__file__).with_name('contract_probe_fixture.py')),
            '--profile', str(profile), '--variant', variant], os.environ, fixture_log)
        deadline = time.monotonic() + 10
        while fixture.poll() is None and time.monotonic() < deadline:
            if 'contract fixture ready' in fixture_log.read_text():
                break
            time.sleep(0.05)
        assert 'contract fixture ready' in fixture_log.read_text(), fixture_log.read_text()
        executable = Path(get_package_prefix('nav_perception_iface')) / (
            'lib/nav_perception_iface/nav_interface_check')
        checker = OwnedProcess([
            str(executable), '--profile', str(profile), '--sample-sec', '2',
            '--wait-timeout', '3'], os.environ, checker_log)
        if variant == 'interrupt':
            deadline = time.monotonic() + 10
            while checker.poll() is None and time.monotonic() < deadline:
                if '等待在 RViz' in checker_log.read_text():
                    break
                time.sleep(0.05)
            assert '等待在 RViz' in checker_log.read_text()
            # OwnedProcess pins the unreaped leader identity until close().
            os.kill(checker.pid, signal.SIGINT)
        deadline = time.monotonic() + 12
        while checker.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        assert checker.poll() == expected, checker_log.read_text()
        assert fixture.poll() is None, 'observer must not stop or signal a provider'
        reports = [json.loads(line) for line in checker_log.read_text().splitlines()
                   if line.startswith('{')]
        assert len(reports) == 1
        report = reports[0]
        if code:
            assert code in [item['code'] for item in report['findings']]
        elif expected == 0:
            assert report['status'] == 'ready' and not report['findings']
            assert '/contract/loc_odom' in report['evidence']['samples']
        else:
            assert report['evidence'] is None
    finally:
        if checker is not None:
            checker.close()
        if fixture is not None:
            fixture.close()
