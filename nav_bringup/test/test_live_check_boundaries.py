"""在线检查说明所声明的比较方向与坐标系捷径（任务 P7.4）."""

from nav_bringup.navcfg.live_check import assess, LiveSnapshot, LiveSpec, TopicRule
from nav_bringup.navcfg.live_ros import RosProbe

SPEC = LiveSpec(
    map_frame='map', odom_frame='odom', nav_frame='nav_base',
    topics=(TopicRule('/cmd', minimum=1, maximum=1),),
    parameters={'/planner': {'tolerance': 0.5}})


def _snapshot(parameters):
    return LiveSnapshot(
        localized=True, tf_age_sec=0.1, publishers={'/cmd': 1}, parameters=parameters,
        node_counts={'/planner': 1}, topic_types={'/cmd': ('geometry_msgs/msg/Twist',)})


def test_extra_runtime_parameters_are_not_reported():
    """比较是单向的：运行时多出来的参数不算问题，只有缺失与不匹配才报."""
    findings = assess(SPEC, _snapshot({'/planner': {'tolerance': 0.5, 'extra': 7}}))

    assert findings == ()


def test_missing_rendered_parameter_is_still_reported():
    """同一方向的另一半：渲染过但运行时没声明的键必须报出来."""
    findings = assess(SPEC, _snapshot({'/planner': {'extra': 7}}))

    assert [finding.code for finding in findings] == ['parameter_missing']


def test_identical_frames_are_localized_without_consulting_tf():
    """两个坐标系配置成同一个时直接算已定位，此时 TF 缓冲区还没建立."""
    probe = RosProbe(LiveSpec(
        map_frame='map', odom_frame='map', nav_frame='nav_base', topics=(), parameters={}))

    assert probe.buffer is None
    assert probe.localized() is True
