# Hardware fact source locator

The single editable board-level hardware fact source is the `硬件.md` associated with the active project.

This file intentionally does not copy its pin map, power conclusions, module parameters, or unresolved items. Keeping a second table inside the Skill would allow the two sources to drift.

## Discovery contract

1. Start at the active project or workspace root.
2. Check that directory and walk upward for `硬件.md`; do not depend only on a fixed relative path or a Git worktree.
3. In the standard repo-scoped layout, the expected file is [`../../../../硬件.md`](../../../../硬件.md).
4. If multiple candidates could govern the active project, resolve the project boundary before making hardware-related changes.
5. If no candidate exists, report the missing hardware fact source and do not reconstruct it from the derived summaries in this Skill.

When a confirmed board fact changes, edit the discovered root `硬件.md` only. Update derived summaries only when the changed fact affects them. Use bundled board schematics and TI documents to verify proposals, but surface discrepancies instead of silently overriding the root fact source.
