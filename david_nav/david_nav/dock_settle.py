"""Compatibility exports for nav_tasks.skills.docking.settle; new logic belongs there."""

from nav_tasks.skills.docking.settle import (
    SettleSpec, is_settled, wait_until_settled,
)

__all__ = [
    'SettleSpec',
    'is_settled',
    'wait_until_settled',
]
