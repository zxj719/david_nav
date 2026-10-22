---
name: refactor-task
description: Executes one approved refactoring or slimming change on the David navigation stack (david_nav) in an isolated workspace, with test-first fixes, test-inventory diffs for deletions, clean-baseline comparisons and an explicit stop at field-evidence boundaries. Use when asked to refactor, slim down, simplify, delete mechanisms, restructure packages, or run "重构/减法/轻量化" work in this repository.
---

# Refactor task (david_nav)

Background and the incidents behind every rule: [docs/refactor_playbook.md](../../../docs/refactor_playbook.md).
Templates and David commands: [REFERENCE.md](REFERENCE.md).

## Hard stops — never cross these

- **No scheduled or unattended loops** (/loop, cron, "every 4 h"). One invocation = one approved change ticket, then stop.
- **No motion**: never send goals or velocities to the real robot; never start/stop localization (alias 3).
- **Never edit the deployed tree** (`~/ros2_ws_refactor`, symlink-install = live behavior). Work only in an isolated clone + workspace.
- **Missing field evidence → write `WAITING_FIELD` and stop.** Do not fill the wait with extra tests, docs, gates or abstractions.
- **Never invert a test that encodes a field finding** without new field evidence in a separate commit.
- **No new framework** (config language, event bus, plugin registry) without a named second real consumer.
- Never print or record credentials (`.bashrc` contains some): grep only the alias lines you need, redact first.

## Workflow

Copy this checklist into your progress notes and tick it off:

```
- [ ] 1. Ticket: approved change ticket exists (type, scope, must-keep behavior, evidence, rollback) — else ask
- [ ] 2. Isolated workspace: clone + build; baseline full test on a CLEAN checkout; collect test IDs
- [ ] 3. Implement: fixes test-first (watch it fail); deletions remove mechanism + its own tests together
- [ ] 4. Inventory diff for deletions: unexpected removals = 0
- [ ] 5. Offline gate: full colcon test 0 failures; sandbox with explicit workspace; new failures reproduced on pristine baseline
- [ ] 6. Commit: one commit per ticket type (fix vs simplification), 3-part message, real author identity
- [ ] 7. Field handoff: step-by-step sheet + records template; status WAITING_FIELD
- [ ] 8. Record status in PROGRESS (single source of truth); report offline vs field results separately
```

### Step notes

1. **Ticket type decides the rules.** *Defect fix* may change behavior (name the error it fixes). *Structural simplification* must not change accepted behavior. Never mix both in one commit.
2. **Baseline on a clean checkout**, never on the deployed tree — leftover ignored files made a failing test look green for a month.
3. **Test-first for behavior.** A test that passes before the change proves nothing.
4. **Deletion inventory**: `.claude/skills/refactor-task/scripts/test_inventory.py collect …` before and after, then `diff --allow-file …`; every removed test must be a gate you meant to delete or a test of the deleted module. Behavior tests are migrated, not deleted.
5. **Sandbox selects its workspace by variable**: nav_bringup uses `REFACTOR_WS`, nav_sim uses `SANDBOX_INSTALL_SETUP`; verify `ros2 pkg prefix nav_bringup` points at your workspace. A failure is "pre-existing" only after reproducing it on a pristine baseline build.
6. **Commit message** = what was removed/changed · what must not change · evidence (test names, counts).
7. **Field sheet** must cover: restart + health-check localization, open test spot (≥ footprint + inflation clearance), goals ≥ 3× goal tolerance, out-and-back legs, cancel, ≥ 10 min run, per-trial table. Thresholds are calibrated by field data, not offline guesses.

## Tripwires — stop and ask a human

- New test lines > 2× the production lines changed by the ticket.
- More than one docs/comment-only commit in a row.
- A fix needs a second fix in a different place (third attempt = question the design).
- Anything requires starting the robot's stack, pushing, merging to `main` or switching aliases.

## Done means

Clean full test green · zero new sandbox failures · field scenarios passed with per-trial records · no new generic framework · changing one Nav2 parameter, stopping from another terminal and diagnosing one localization fault take no more steps than before.
