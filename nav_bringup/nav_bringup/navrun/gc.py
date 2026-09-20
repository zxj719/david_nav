"""显式的保留策略命令行，复用共享的"已完成记录"策略与安全删除器.

只处理已经完成的记录：正在进行的运行不会被清理。
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import datetime
import json

from nav_bringup.navrun.locking import LockBusy, RunLock
from nav_bringup.recorder import retention, run_dir, safe_retention


def run(args: argparse.Namespace) -> int:
    """预览或执行保留策略，全程不启动 ROS，也不启动任何导航进程.

    Args:
        args: 运行记录根目录、私有/导航租约、dry-run 标志与策略上限。

    Returns:
        无事可做、预览完成或执行完成返回 0；输入非法、租约被占或失败返回 2。
        删除是逐条推进的，不是事务：中途失败会留下已删与未删并存的状态。
    """
    result = {'dry_run': args.dry_run, 'applied': False, 'pruned_runs': [],
              'deleted_bags': [], 'removed_bytes': 0, 'skipped': {}}
    try:
        policy = retention.RetentionPolicy(args.max_runs, args.max_total_bytes,
                                           args.bag_max_age_days)
        root = (args.runs_root or run_dir.default_runs_root()).expanduser().absolute()
        try:
            with safe_retention.open_root(root):
                pass
        except FileNotFoundError:
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
            return 0
        context = nullcontext() if args.dry_run else RunLock(args.lock_path.expanduser())
        with context:
            plan = retention.plan_gc(run_dir.list_runs(root), policy, datetime.datetime.now())
            result.update(pruned_runs=[item.run_id for item in plan.prune_runs],
                          deleted_bags=[path.parent.name + '/bag' for path in plan.delete_bags],
                          skipped=plan.skipped)
            if not args.dry_run:
                result['removed_bytes'] = retention.apply_gc(plan, root)
                result['applied'] = True
    except (ValueError, OSError, LockBusy) as error:
        result.update(error=type(error).__name__, partial_deletion_possible=not args.dry_run)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        return 2
    except KeyboardInterrupt:
        result.update(error='interrupted', partial_deletion_possible=not args.dry_run)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        return 130
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0
