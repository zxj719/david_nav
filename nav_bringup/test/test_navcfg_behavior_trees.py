"""行为树模板、配置覆盖、旧树逐属性等价和错误输入门禁."""

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from nav_bringup.navcfg import behavior_trees as bt
from nav_bringup.navcfg import cli, render

ROOT = Path(__file__).resolve().parents[2]
PLUGIN = ROOT / 'nav_plugins'
ROBOT = ROOT / 'david_config'
CONFIG = ROOT / 'nav_bringup' / 'config'


def _signature(xml):
    return [(node.tag, node.attrib) for node in ET.fromstring(xml).iter()]


@pytest.mark.parametrize('mode_name', ['tour', 'precise', 'bench_slamgt', 'teleop'])
def test_david_templates_equal_every_legacy_node_and_attribute(mode_name):
    """Each David mode reproduces all three existing trees without semantic normalization."""
    profile = render.load_yaml(ROBOT / 'robot/david.yaml')
    source = ROBOT / 'modes' / f'{mode_name}.yaml'
    mode = render.parse_mode(render.load_yaml(source), source)
    bundle = bt.render_bundle(
        PLUGIN / 'behavior_trees', PLUGIN / 'config/bt_bindings.yaml',
        render.deep_merge(profile, mode.profile))
    originals = ROOT / 'david_nav/behavior_trees'
    assert set(bundle) == {path.name for path in originals.glob('*.xml')}
    for name, xml in bundle.items():
        assert _signature(xml) == _signature((originals / name).read_text(encoding='utf-8'))


def test_frames_and_speeds_come_only_from_the_profile():
    """Another robot changes frames and recovery values without changing templates."""
    profile = render.load_yaml(ROBOT / 'robot/david.yaml')
    profile['frames'].update(nav_base='other_center', map='other_world')
    profile['reactive']['escape'].update(longitudinal_speed=0.12, lateral_speed=0.0)
    bundle = bt.render_bundle(
        PLUGIN / 'behavior_trees', PLUGIN / 'config/bt_bindings.yaml', profile)
    for xml in bundle.values():
        for node in ET.fromstring(xml).iter():
            if 'robot_base_frame' in node.attrib:
                assert node.attrib['robot_base_frame'] == 'other_center'
            if node.tag == 'EscapeObstacle':
                assert node.attrib['longitudinal_speed'] == '0.12'
                assert node.attrib['lateral_speed'] == '0.00'
            if node.tag == 'RemovePassedGoals':
                assert node.attrib['global_frame'] == 'other_world'


def test_xml_escaping_does_not_rewrite_blackboard_values():
    """Attribute escaping protects XML while blackboard braces stay untouched."""
    binding = bt.parse_bindings({'schema_version': 1, 'bindings': {'frame': {'key': 'frame'}}})
    text = ('<root><BehaviorTree ID="Main">'
            '<Action frame="${frame}" goal="{goal}"/></BehaviorTree></root>')
    xml = bt.render_template(text, {'frame': 'a"&<b'}, binding)
    node = ET.fromstring(xml).find('.//Action')
    assert node.attrib == {'frame': 'a"&<b', 'goal': '{goal}'}


def test_legacy_decimal_format_does_not_round_new_tuning_values():
    """Preserving old trailing zeros must not silently round new profile values."""
    binding = bt.parse_bindings({
        'schema_version': 1, 'bindings': {'speed': {'key': 'speed', 'format': '.2f'}}})
    xml = bt.render_template(
        '<root><BehaviorTree ID="Main"><Action speed="${speed}"/></BehaviorTree></root>',
        {'speed': 0.12345}, binding)
    assert ET.fromstring(xml).find('.//Action').attrib['speed'] == '0.12345'


@pytest.mark.parametrize('value', [None, True, [], {}, '', float('nan'), float('inf')])
def test_invalid_attribute_values_are_rejected(value):
    """Empty, structured, boolean and nonfinite values cannot enter BT attributes."""
    binding = bt.parse_bindings({'schema_version': 1, 'bindings': {'value': {'key': 'value'}}})
    with pytest.raises(render.RenderError):
        bt.render_template(
            '<root><BehaviorTree ID="Main"><Action x="${value}"/></BehaviorTree></root>',
            {'value': value}, binding)


@pytest.mark.parametrize('text', [
    '<root><BehaviorTree ID="Main"><Action x="${missing}"/></BehaviorTree></root>',
    '<root><BehaviorTree ID="Main"><Action x="$!"/></BehaviorTree></root>',
    '<root><BehaviorTree ID="Main"></root>', '<other/>',
])
def test_invalid_templates_fail(text):
    """Unknown variables, malformed placeholders and invalid trees fail closed."""
    with pytest.raises(render.RenderError):
        bt.render_template(text, {}, {})


@pytest.mark.parametrize('data', [
    None, {'schema_version': 2, 'bindings': {}},
    {'schema_version': 1, 'bindings': {'invalid-token': {'key': 'frame'}}},
    {'schema_version': 1, 'bindings': {'ok': {'key': 'frame', 'format': '>999999999'}}},
])
def test_invalid_binding_tables_fail(data):
    """Only explicit supported attribute formatters are accepted."""
    with pytest.raises(render.RenderError):
        bt.parse_bindings(data)


def _cli_args(tmp_path):
    return [
        'render', '--profile', str(ROBOT / 'robot/david.yaml'),
        '--mode', str(ROBOT / 'modes/precise.yaml'),
        '--base', str(CONFIG / 'nav2_base.yaml'),
        '--bindings', str(CONFIG / 'nav2_bindings.yaml'),
        '--output', str(tmp_path / 'nav2.yaml'),
        '--bt-output-dir', str(tmp_path / 'bt'),
        '--bt-template-dir', str(PLUGIN / 'behavior_trees'),
        '--bt-bindings', str(PLUGIN / 'config/bt_bindings.yaml'),
    ]


def test_cli_writes_bt_files_and_absolute_nav2_paths(tmp_path):
    """The runnable Nav2 output points at this run's rendered XML, not old package data."""
    assert cli.main(_cli_args(tmp_path)) == 0
    data = yaml.safe_load((tmp_path / 'nav2.yaml').read_text())
    params = data['bt_navigator']['ros__parameters']
    for key in bt.BT_PARAMETERS:
        path = Path(params[key])
        assert path.is_absolute() and path.parent == tmp_path / 'bt'
        assert path.is_file()


def test_bad_bt_input_writes_neither_parameters_nor_xml(tmp_path):
    """Missing profile keys are diagnosed before creating any output artifacts."""
    profile = copy.deepcopy(render.load_yaml(ROBOT / 'robot/david.yaml'))
    del profile['reactive']['escape']['lateral_speed']
    source = tmp_path / 'incomplete.yaml'
    source.write_text(yaml.safe_dump(profile), encoding='utf-8')
    assert cli.main(_cli_args(tmp_path) + ['--profile', str(source)]) == 2
    assert not (tmp_path / 'nav2.yaml').exists()
    assert not (tmp_path / 'bt').exists()


def test_cli_applies_frame_override_to_bt_and_nav2_together(tmp_path):
    """A mode overlay cannot leave the XML frame behind while changing the Nav2 frame."""
    overlay = render.load_yaml(ROBOT / 'modes/precise.yaml')
    overlay['profile'].setdefault('frames', {})['nav_base'] = 'mode_rotation_center'
    path = tmp_path / 'mode.yaml'
    path.write_text(yaml.safe_dump(overlay), encoding='utf-8')
    assert cli.main(_cli_args(tmp_path) + ['--mode', str(path)]) == 0
    data = yaml.safe_load((tmp_path / 'nav2.yaml').read_text())
    params = data['bt_navigator']['ros__parameters']
    assert params['robot_base_frame'] == 'mode_rotation_center'
    root = ET.parse(params['default_nav_to_pose_bt_xml']).getroot()
    assert root.find('.//EscapeObstacle').attrib['robot_base_frame'] == 'mode_rotation_center'
