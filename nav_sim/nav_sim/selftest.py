"""Optional navrun selftest feature; preparation is distinct from runtime authority."""

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
    """Prepare or explicitly execute a private selftest without host preflight mutations.

    Args:
        args: Parsed robot/mode, output, discovery, deadline and authority flags.

    Returns:
        Zero for the explicitly requested kind passing; one for a failed experiment,
        two for invalid selectors/authority/preparation, or 130 for interruption.
        Only closed_loop results with complete=true prove full selftest completion.

    Raises:
        asyncio.CancelledError: Cancellation is propagated after owned cleanup.
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
