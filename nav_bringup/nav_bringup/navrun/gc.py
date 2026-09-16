"""Explicit retention CLI using the shared completed-record policy and safe remover."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import datetime
import json

from nav_bringup.navrun.locking import LockBusy, RunLock
from nav_bringup.recorder import retention, run_dir, safe_retention


def run(args: argparse.Namespace) -> int:
    """Preview or apply retention without ever starting ROS or a navigation process.

    Args:
        args: Runs root, private/navigation lease, dry-run flag and policy limits.

    Returns:
        Zero for a valid no-op, preview or completed application; two for invalid
        inputs, busy leases or failure. Deletion is incremental, not a transaction.
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
