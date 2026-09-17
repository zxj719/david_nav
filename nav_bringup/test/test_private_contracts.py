"""保护短小但非直观的进程与文件安全边界说明."""

import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('doctor/checks.py', '_segment_base'),
    ('doctor/checks.py', '_tree_size'),
    ('doctor/live.py', '_endpoints'),
    ('lint/architecture.py', '_adapter'),
    ('lint/docstrings.py', '_is_property'),
    ('lint/docstrings.py', '_contains'),
    ('lint/docstrings.py', '_annotation_is_none'),
    ('lint/node_attributes.py', '_qualified'),
    ('navcfg/documentation.py', '_type_name'),
    ('navcfg/documentation.py', '_requirement'),
    ('navcfg/documentation.py', '_constraints'),
    ('navcfg/documentation.py', '_bound'),
    ('navcfg/documentation.py', '_value'),
    ('navcfg/documentation.py', '_cell'),
    ('navrun/check.py', '_root'),
    ('navrun/check.py', '_catalog'),
    ('navrun/dds_hygiene.py', '_group'),
    ('navrun/dds_hygiene.py', '_state'),
    ('navrun/preparation.py', '_path'),
    ('navrun/preparation.py', '_yaml'),
    ('navrun/preparation.py', '_json'),
    ('navrun/prepared_parameters.py', '_node'),
    ('navrun/recorded_startup.py', '_event'),
    ('launching/component_loader.py', 'CheckedComponentLoader._poll'),
    ('lint/docstrings.py', '_iter_python_files'),
    ('lint/docstrings.py', '_split_sections'),
    ('lint/docstrings.py', '_documentable_args'),
    ('lint/docstrings.py', '_returns_value'),
    ('lint/docstrings.py', '_raises_explicitly'),
    ('lint/node_attributes.py', '_class_statements'),
    ('lint/node_attributes.py', '_delegates'),
    ('navcfg/live_check.py', '_equal'),
    ('navcfg/live_ros.py', 'RosProbe._counts'),
    ('navcfg/render.py', '_index'),
    ('navcfg/render.py', '_child'),
    ('navcfg/render.py', '_format_footprint_string'),
    ('navcfg/schema.py', '_parse_range'),
    ('navcfg/schema.py', '_ignored_by_colcon'),
    ('navcfg/validation.py', '_inverse'),
    ('navrun/cli.py', '_run_directory'),
    ('navrun/control_client.py', '_peer'),
    ('navrun/executor_binding.py', '_path'),
    ('navrun/executor_binding.py', '_duration'),
    ('navrun/preflight.py', '_patterns'),
    ('navrun/preflight.py', '_is_launch_wrapper'),
    ('navrun/process_backend.py', '_read'),
    ('navrun/recorded_startup.py', '_Signals._handle'),
    ('navrun/recorded_startup.py', '_check_signal'),
    ('navrun/recorded_startup.py', '_summary'),
    ('navrun/records.py', '_mapping'),
    ('navrun/wrap.py', '_naive_local'),
    ('navrun/wrap.py', '_tee'),
    ('navrun/wrap.py', '_write_all'),
    ('recorder/manifest.py', '_aware'),
    ('recorder/manifest.py', '_iso'),
    ('recorder/nav_events.py', '_nonempty'),
    ('recorder/resources.py', '_aggregate'),
    ('recorder/run_event_stream.py', 'RunEventStream._warning'),
    ('recorder/summary.py', '_duration'),
    ('recorder/summary.py', '_clock_time'),
    ('recorder/topic_stats.py', '_row'),
    ('navrun/dds_hygiene.py', '_pid_entries'),
    ('navrun/dds_hygiene.py', '_directory'),
    ('navrun/owned_process.py', 'OwnedProcess._check_owner'),
    ('navrun/owned_process.py', 'OwnedProcess._signal'),
    ('navrun/preparation.py', '_publish_marker'),
    ('navrun/record_reader.py', 'RecordReader._directory'),
    ('navrun/recorded_startup.py', '_check_plan'),
    ('navrun/recorded_startup.py', '_write_manifest'),
    ('navrun/recorded_startup.py', '_gc'),
    ('navrun/wrap.py', '_SignalRelay._handle'),
    ('recorder/linked_process.py', '_unique_target'),
    ('recorder/nav_events.py', '_pairs'),
    ('recorder/retention.py', '_require_run_under'),
    ('recorder/sessions.py', '_signal_group'),
])
def test_nonobvious_private_contract_has_documentation(path, qualified_name):
    """按名称保留说明要求，不把函数长度或说明存在当作安全证明.

    Args:
        path: 相对于包源码目录的模块路径。
        qualified_name: 类与函数组成的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_bringup'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in node.body
                   if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                   and child.name == name]
        assert len(matches) == 1, qualified_name
        node = matches[0]
    assert isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)), qualified_name
    assert (ast.get_docstring(node) or '').strip(), qualified_name
