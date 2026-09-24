"""Validate shared environment assets without starting ROS or changing the host."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
INSTALLED_XML = (
    '/home/agx/ros2_ws_refactor/install/david_system_env/share/'
    'david_system_env/config/fastdds_david.xml')


@pytest.mark.parametrize('domain', [None, '99', '0'])
def test_loader_exports_profiles_and_keeps_external_domain_precedence(domain):
    """The loader exports DDS paths without overriding an existing domain."""
    environment = {'PATH': '/usr/bin:/bin', 'LC_ALL': 'C'}
    if domain is not None:
        environment['ROS_DOMAIN_ID'] = domain
    loader = ROOT / 'config/load_ros_env.sh'
    result = subprocess.run(
        ['/bin/bash', '--noprofile', '--norc', '-c',
         'set -u; source "$1"; /usr/bin/env', 'asset-test', str(loader)],
        env=environment, capture_output=True, text=True, timeout=5, check=True)
    exported = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    assert exported['ROS_DOMAIN_ID'] == (domain if domain is not None else '0')
    assert exported['FASTRTPS_DEFAULT_PROFILES_FILE'] == INSTALLED_XML
    assert not any(name.startswith('__david_ros_env_') for name in exported)


def test_systemd_environment_file_stays_literal_data():
    """The service environment file contains only literal assignments."""
    lines = (ROOT / 'config/ros_env.conf').read_text().splitlines()
    assignments = [line for line in lines if line.strip() and not line.lstrip().startswith('#')]
    assert assignments == ['ROS_DOMAIN_ID=0', 'FASTRTPS_DEFAULT_PROFILES_FILE=' + INSTALLED_XML]


def test_xml_keeps_loopback_and_shared_memory_transports():
    """The single DDS profile keeps the validated local transports."""
    tree = ET.parse(ROOT / 'config/fastdds_david.xml')
    ns = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    addresses = tree.findall('.//dds:interfaceWhiteList/dds:address', ns)
    assert [item.text for item in addresses] == ['127.0.0.1']
    assert [item.text for item in tree.findall('.//dds:type', ns)] == ['SHM', 'UDPv4']
    assert tree.find('.//dds:useBuiltinTransports', ns).text == 'false'


def test_package_does_not_auto_source_an_environment_hook():
    """Installing the package does not silently mutate every sourced shell."""
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    assert 'ament_environment_hooks' not in cmake
    assert 'install(DIRECTORY config DESTINATION' in cmake
