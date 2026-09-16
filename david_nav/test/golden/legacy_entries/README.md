# Frozen entry-point references

These are byte-identical snapshots from refactor commit `87c5fc9`, captured before
replacing the old entries with compatibility shims. They are test data, not
installed modules or supported runtime entry points. Do not regenerate them from
the new implementation when changing a shim or resolving a compatibility failure.

| Fixture | Original source | Last source change | SHA-256 |
|---|---|---|---|
| navigation_executor.py.txt | david_nav/david_nav/navigation_executor.py | 8c4861b | 5f7bdf437c06ba886d4d23812c0767a336de2535a97503b343b75b2800d2da53 |
| dock_navigator.py.txt | david_nav/david_nav/dock_navigator.py | 35cdda7 | cf5883eb3e3b468a15bedb90c70d12f31c2f56f4aad5d78f760b39f22a8314c6 |

The `.py.txt` extension keeps these unmodified historical sources out of Python
package discovery, automatic pytest collection and production installation.
Tests load selected pure AST methods or use runpy with a non-main module name;
never run either snapshot as a program. No ROS initialization or goals are needed.

The nav_executor CLI golden compares defaults, flags and exact help against the
executor snapshot. nav_tasks compares Dock workflow/report/event logs, run_action
audit branches and the sixteen parameter declarations against the Dock snapshot.
The Dock reference intentionally includes U2's static footprint guard and 0.05
rad/s threshold; it is not the pre-U2 production snapshot. Existing robot-specific
defaults and historical defects remain here as evidence, not recommendations.

Integrity tests pin the bytes independently of the live source paths. Golden
comparisons still have their existing scope: this freeze does not prove physical
motion behavior, future legacy-shim wiring, or compatibility of untested branches.
