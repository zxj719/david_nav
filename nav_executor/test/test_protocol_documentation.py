"""Keep protocol documentation tied to the unchanged codec and wire goldens."""

import ast
import hashlib
import json
from pathlib import Path
import re

import pytest
import yaml

from nav_executor.protocol import codec
from nav_tasks.api import PoseRecord, StoredPoi

ROOT = Path(__file__).resolve().parents[2]
DOCUMENT = ROOT / 'docs/protocol/nav_ws_v1.asyncapi.yaml'
META_SCHEMA = ROOT / 'docs/protocol/schemas/asyncapi-2.6.0.json'
META_SCHEMA_SHA256 = 'be227ff668d64da62932c8409daca58fef1f7749f5d386175111f17314595137'
COMMANDS = ('NavigateTo', 'MarkCurrentPoi', 'AbortNavigation', 'UpdatePoiList')
EVENTS = ('Progress', 'Arrived', 'Error', 'MarkPoiAck', 'MarkPoiSuccess', 'MarkPoiError')


def _document():
    return yaml.safe_load(DOCUMENT.read_text(encoding='utf-8'))


def _resolve(document, reference):
    assert reference.startswith('#/'), 'Protocol documentation must not fetch remote schemas'
    result = document
    for name in reference[2:].split('/'):
        result = result[name.replace('~1', '/').replace('~0', '~')]
    return result


def _payload(document, name):
    message = document['components']['messages'][name]
    return _resolve(document, message['payload']['$ref'])


def _examples(document):
    for name, message in document['components']['messages'].items():
        for example in message['examples']:
            yield name, example['payload']


def _golden_frames():
    # Read existing literal expectations; do not regenerate or execute reference runtimes.
    paths = [ROOT / 'david_nav/test/test_executor_golden.py',
             ROOT / 'nav_executor/test/test_task_wire_golden.py']
    frames = []
    for path in paths:
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                try:
                    value = json.loads(node.value)
                except (ValueError, TypeError):
                    continue
                if isinstance(value, dict) and 'event_type' in value:
                    frames.append(value)
    assert len(frames) >= 10, 'Expected existing complete literal wire goldens'
    return frames


def test_asyncapi_direction_and_message_inventory():
    """Check AsyncAPI 2.6 directions from the executor application's perspective."""
    document = _document()
    assert document['asyncapi'] == '2.6.0'
    channel = document['channels']['/']
    expected = {'publish': set(COMMANDS) | {'Ping', 'Pong'},
                'subscribe': set(EVENTS) | {'Ping', 'Pong'}}
    for direction, names in expected.items():
        refs = channel[direction]['message']['oneOf']
        assert {item['$ref'].split('/')[-1] for item in refs} == names
        for item in refs:
            assert _resolve(document, item['$ref'])
    assert set(document['components']['messages']) == set(COMMANDS + EVENTS + ('Ping', 'Pong'))
    for name, frame in _examples(document):
        schema = _payload(document, name)
        assert set(schema['required']) <= frame.keys()
        assert set(schema['properties']) == set(schema['x-wire-keys'])
    for name in COMMANDS:
        for example in document['components']['messages'][name]['examples']:
            assert codec.parse_command(example['payload']).action == (
                _payload(document, name)['properties']['action']['const'])


def test_outbound_documentation_matches_actual_builders_and_order():
    """Every canonical output example is built by the real ordered codec."""
    document = _document()
    pose = PoseRecord('fixture', 0, 0, 0, 0, 0)
    poi = StoredPoi('POI_001', '前台', pose, pose, 'fixture')
    frames = {
        'Progress': codec.build_progress_event('req_1', 1, 2.3456, 'running'),
        'Arrived': codec.build_arrived_event('req_1', 1),
        'Error': codec.build_error_event('req_1', 1, '目标点不存在: POI_404'),
        'MarkPoiAck': codec.build_mark_poi_ack('req_1', 1),
        'MarkPoiSuccess': codec.build_mark_poi_success_event('req_1', 1, poi),
        'MarkPoiError': codec.build_mark_poi_error('req_1', 1, '位姿不可用'),
    }
    for name, frame in frames.items():
        example = document['components']['messages'][name]['examples'][0]['payload']
        assert json.dumps(example, ensure_ascii=False) == json.dumps(frame, ensure_ascii=False)
        schema = _payload(document, name)
        assert list(frame) == schema['x-wire-keys']
        assert set(schema['required']) == frame.keys()
        assert schema['additionalProperties'] is False


def test_all_frozen_event_literals_have_a_documented_message():
    """The sample mapping covers every unchanged event frame in both golden suites."""
    document = _document()
    types = {_payload(document, name)['properties']['event_type']['const']: name
             for name in EVENTS}
    frames = _golden_frames()
    for frame in frames:
        schema = _payload(document, types[frame['event_type']])
        assert list(frame) == schema['x-wire-keys']
        assert set(schema['required']) == frame.keys()


def _expand(document, value, active=()):
    if isinstance(value, list):
        return [_expand(document, item, active) for item in value]
    if not isinstance(value, dict):
        return value
    if '$ref' in value:
        reference = value['$ref']
        assert reference not in active, 'No cyclic payload references in this contract'
        assert len(value) == 1, 'Do not silently drop reference siblings'
        return _expand(document, _resolve(document, reference), active + (reference,))
    return {key: _expand(document, item, active) for key, item in value.items()}


def test_payloads_and_existing_goldens_with_json_schema():
    """Run a real Draft 7 validator when available; do not substitute a subset checker."""
    jsonschema = pytest.importorskip(
        'jsonschema', reason='python3-jsonschema is declared in package.xml but not installed')
    document = _document()
    validators = {}
    for name in document['components']['messages']:
        schema = _expand(document, _payload(document, name))
        jsonschema.Draft7Validator.check_schema(schema)
        validators[name] = jsonschema.Draft7Validator(schema)
    for name, frame in _examples(document):
        validators[name].validate(frame)
    types = {_payload(document, name)['properties']['event_type']['const']: name
             for name in EVENTS}
    for frame in _golden_frames():
        validators[types[frame['event_type']]].validate(frame)
    # Ensure the validator is enforcing required fields/types, not just loading YAML.
    for name, frame in _examples(document):
        schema = _payload(document, name)
        missing = dict(frame)
        del missing[schema['required'][0]]
        assert not validators[name].is_valid(missing)
        assert not validators[name].is_valid([])


def test_document_matches_the_official_asyncapi_meta_schema():
    """用归档的官方 2.6.0 元 schema 校验整份文档，而不是只核对我们自己挑的字段.

    先核对归档文件的 sha256（来源与固定提交见 docs/protocol/schemas/README.md），
    再用 draft-07 校验；最后用一份残缺文档确认这个校验器真的会拒绝，不是空转。
    """
    jsonschema = pytest.importorskip(
        'jsonschema', reason='python3-jsonschema is declared in package.xml but not installed')
    raw = META_SCHEMA.read_bytes()

    assert hashlib.sha256(raw).hexdigest() == META_SCHEMA_SHA256, (
        'vendored meta schema changed; update docs/protocol/schemas/README.md first')
    schema = json.loads(raw.decode('utf-8'))
    assert schema['$schema'] == 'http://json-schema.org/draft-07/schema'
    jsonschema.Draft7Validator.check_schema(schema)
    validator = jsonschema.Draft7Validator(schema)

    assert list(validator.iter_errors(_document())) == []
    assert not validator.is_valid({'asyncapi': '2.6.0'})


@pytest.mark.parametrize('relative', [
    'docs/protocol/nav_ws_v1.md', 'docs/frozen_logs.md',
    'docs/adr/0001-navigation-runs-as-background-task.md',
    'docs/adr/0002-reliable-outbox.md', 'docs/adr/0004-navigation-retry.md',
    'docs/adr/0007-named-poses-source.md',
])
def test_contract_documents_exist_and_local_links_resolve(relative):
    """Named deliverables cannot be replaced by stale references or missing files."""
    path = ROOT / relative
    source = path.read_text(encoding='utf-8')
    assert len(source) > 300
    for target in re.findall(r'\]\(([^)]+)\)', source):
        if '://' in target or target.startswith('#'):
            continue
        assert (path.parent / target.split('#')[0]).exists(), (relative, target)


def test_frozen_log_document_covers_actual_bench_regex_consumers():
    """Every current Dock timeline regex and the SPEC log families remain listed."""
    document = (ROOT / 'docs/frozen_logs.md').read_text(encoding='utf-8')
    consumer = ROOT / 'david_nav/scripts/dock_bench/dock_common.py'
    names = {node.id for node in ast.walk(ast.parse(consumer.read_text(encoding='utf-8')))
             if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store)
             and node.id.startswith('_RE_')}
    assert len(names) >= 7
    assert all(name in document for name in names)
    assert all(token in document for token in (
        'NAV ERROR reported to server:', 'STATUS state=', 'OUT RESENT(断联重发)',
        'NAV FAILURE attempt', 'parse_dock_timeline', '_VERDICT_RULES'))
