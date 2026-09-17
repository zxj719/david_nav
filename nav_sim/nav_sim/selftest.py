"""提供可选 navrun 自测入口，离线准备、启动验证与闭环运行权限分开处理."""

import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
import tempfile
import uuid

from nav_bringup.navcfg import schema
from nav_bringup.navrun.up_planning import discover_inputs, UpRequest
from nav_sim.model import finite_number, require_sandbox
from nav_sim.selftest_scene import prepare_scene


def run(args: argparse.Namespace) -> int:
    """按显式选项准备或执行私有自测，不执行主机提权或清理既有栈.

    prepare_only 优先只生成场景；其成功不证明 ROS 图启动或导航完成。
    完整闭环仍需外部运动授权，入口开关不能覆盖现场安全约定。

    Args:
        args: 已解析的机器人、模式、输出、资源查找、超时及运行开关。

    Returns:
        所选流程通过为 0，实验失败为 1，捕获的选择/准备错误为 2，
        键盘中断为 130。只有 closed_loop 且 complete=true 才表示
        本自测判据通过，不等同于实机验收。

    Raises:
        asyncio.CancelledError: 运行阶段取消向外传播；已进入的资源收尾由运行器负责。
    """
    try:
        if finite_number(args.timeout, 'timeout') <= 0:
            raise ValueError('timeout must be positive')
        if not any((args.prepare_only, args.startup_only, args.allow_goals)):
            raise ValueError('select --prepare-only, --startup-only or explicit --allow-goals')
        if not args.prepare_only:
            require_sandbox(os.environ)
            from nav_sim.selftest_runtime import execute, require_idle_host

            require_idle_host()
        # mkdtemp owns only a private container; prepare_scene requires its child
        # to be new. A failed render retains its partial assets for diagnosis.
        root = (args.output_dir.expanduser().absolute() if args.output_dir is not None else
                Path(tempfile.mkdtemp(prefix='nav_selftest_')) / 'run')

        def share(package):
            if args.source_root is not None:
                path = args.source_root.expanduser().absolute() / package
                if not (path / 'package.xml').is_file():
                    raise LookupError('package resources unavailable: ' + package)
                return path
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory(package))

        inputs = discover_inputs(UpRequest(args.mode, args.robot, 'selftest', root), share)
        paths = (schema.find_source_schemas(args.source_root.expanduser())
                 if args.source_root is not None else schema.find_installed_schemas(
                     schema.ament_prefixes(os.environ)))
        catalog = schema.load_catalog(paths)
        namespace = 'selftest_' + uuid.uuid4().hex

        def prepare(uri):
            return prepare_scene(root, args.robot, args.mode, namespace, inputs, catalog, uri)

        if args.prepare_only:
            # Placeholder URI is not bound or executed. Runtime always prepares a
            # new directory using the port of its actual owned loopback listener.
            scene = prepare('ws://127.0.0.1:18101')
            result = dict(kind='prepared', executed=False, complete=False,
                          plan_sha256=scene.digest)
            status = 0
        else:
            result = dict(asyncio.run(execute(
                prepare, args.timeout, args.startup_only, args.allow_goals)))
            status = 0 if result['passed'] else 1
        result['run_dir'] = str(root)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
        return status
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError, LookupError, ImportError) as error:
        print('selftest: ' + str(error), file=sys.stderr)
        return 2
