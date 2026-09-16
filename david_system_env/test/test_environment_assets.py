"""Freeze the shared environment migration without starting ROS or changing the host."""

import hashlib
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent
HASHES = {
    'ros_env.conf': '14abf6a1d2b1dd4f27068687ae92f7db878be534c0c39b7a5f8bfe982eb503a1',
    'load_ros_env.sh': '2bd9becd7c99ee91b6ced39671efdb1b9605580c748c583bffe481df1b53006f',
    'fastdds_david.xml': '671f5587e5d256d3a652d58f129a2a428d339397b2c84ad9247c99340a3fb50c',
}


@pytest.mark.parametrize('name', sorted(HASHES))
def test_canonical_assets_preserve_pre_migration_bytes(name):
    """Moving ownership must not change domain, whitelist, transport, paths or shell behavior."""
    assert hashlib.sha256((ROOT / 'config' / name).read_bytes()).hexdigest() == HASHES[name]


@pytest.mark.parametrize('name', sorted(HASHES))
def test_old_source_paths_are_relative_links_to_the_new_owner(name):
    """Existing source callers retain their path while each asset has only one owner."""
    old = SOURCE / 'david_nav/config' / name
    assert old.is_symlink()
    assert os.readlink(old) == '../../david_system_env/config/' + name
    assert old.resolve() == ROOT / 'config' / name


@pytest.mark.parametrize('location', ['david_nav', 'david_system_env'])
@pytest.mark.parametrize('domain', [None, '99', '0'])
def test_both_loaders_export_profiles_and_keep_external_domain_precedence(location, domain):
    """An isolated shell sources data only, never ROS setup, launch scripts or providers."""
    environment = {'PATH': '/usr/bin:/bin', 'LC_ALL': 'C'}
    if domain is not None:
        environment['ROS_DOMAIN_ID'] = domain
    loader = SOURCE / location / 'config/load_ros_env.sh'
    result = subprocess.run(
        ['/bin/bash', '--noprofile', '--norc', '-c',
         'set -u; source "$1"; /usr/bin/env', 'asset-test', str(loader)],
        env=environment, capture_output=True, text=True, timeout=5, check=True)
    exported = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    assert exported['ROS_DOMAIN_ID'] == (domain if domain is not None else '0')
    assert exported['FASTRTPS_DEFAULT_PROFILES_FILE'] == (
        '/home/agx/ros2_ws/src/david_nav/config/fastdds_david.xml')
    assert not any(name.startswith('__david_ros_env_') for name in exported)


def test_systemd_environment_file_stays_literal_data():
    """The shared config remains systemd-compatible KEY=value, without shell expansion."""
    lines = (ROOT / 'config/ros_env.conf').read_text().splitlines()
    assignments = [line for line in lines if line.strip() and not line.lstrip().startswith('#')]
    assert assignments == [
        'ROS_DOMAIN_ID=0',
        'FASTRTPS_DEFAULT_PROFILES_FILE=/home/agx/ros2_ws/src/david_nav/config/fastdds_david.xml',
    ]


def test_refactor_xml_keeps_its_existing_loopback_and_shared_memory_transports():
    """This is not permission to widen DDS access or rewrite production's transport policy."""
    tree = ET.parse(ROOT / 'config/fastdds_david.xml')
    ns = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    addresses = tree.findall('.//dds:interfaceWhiteList/dds:address', ns)
    assert [item.text for item in addresses] == ['127.0.0.1']
    assert [item.text for item in tree.findall('.//dds:type', ns)] == ['SHM', 'UDPv4']
    assert tree.find('.//dds:useBuiltinTransports', ns).text == 'false'


def test_legacy_package_declares_shared_data_runtime_dependency():
    """Package installation cannot silently omit the owner of the compatibility links."""
    package = ET.parse(SOURCE / 'david_nav/package.xml')
    assert 'david_system_env' in [item.text for item in package.findall('exec_depend')]
    assert 'david_system_env' in [item.text for item in package.findall('build_depend')]


def test_legacy_install_resolves_shared_files_before_copying_or_linking():
    """Isolated install prefixes must not inherit source-relative cross-package links."""
    cmake = (SOURCE / 'david_nav/CMakeLists.txt').read_text()
    assert 'find_package(david_system_env REQUIRED)' in cmake
    assert '"${david_system_env_DIR}/../config/${env_asset}" REALPATH' in cmake
    assert 'install(FILES "${shared_env_file}" DESTINATION share/${PROJECT_NAME}/config)' in cmake
    for name in HASHES:
        assert 'PATTERN "' + name + '" EXCLUDE' in cmake


def test_shared_data_package_does_not_auto_source_an_environment_hook():
    """Sourcing the workspace must not override another robot or start any participant."""
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    assert 'ament_environment_hooks' not in cmake
    assert 'install(DIRECTORY config docs' in cmake
