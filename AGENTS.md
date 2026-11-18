# Agent entry point

Start with [docs/README.md](docs/README.md) for architecture, behavior to preserve,
validation limits and open issues. Keep project context in [docs/](docs/README.md).
Before creating or changing any desktop tab, window, or UI, read and follow
[docs/desktop-style.md](docs/desktop-style.md).

Before adding helpers or dependencies between modules, read
[docs/reusability.md](docs/reusability.md) and check the linked source for an
existing owner to reuse. Prefer small modules with one clear responsibility and
explicit dependencies; avoid duplicated behavior, generic utility buckets and
wrappers that only forward calls. Keep code concise without removing validation,
error handling, resource cleanup or hardware safeguards.

When shared functions/classes are added, moved, changed or removed, update the
reuse map in the same commit with source links, responsibility, inputs/outputs
and units, a caller/usage example, dependency/lifetime constraints and replaced
APIs. Keep proposed APIs in issues until implemented. See the
[refactoring review](docs/refactor-review.md) for the implementation backlog.

Use the judge/developer workflow only when the user explicitly invokes `$work`
or selects it through `/skills`. Ordinary requests and follow-ups do not enable it.

## Issue workflow

For substantial tasks, create a GitHub issue before implementation with the
problem and acceptance criteria. Small, self-contained tasks do not need an
issue. When an issue is used, implement against it and include its reference in
the commit message. Include a concise summary of the change in the commit body
or an issue comment. Do not close issues unless the user explicitly asks you to
close them. If issue creation is unavailable for a substantial task, report the
blocker and do not claim the workflow is complete.

## GitHub tasks

Use the GitHub CLI (`gh`) for GitHub tasks when the needed operation is
supported, including issues, pull requests, reviews and repository queries.
Use browser workflows only when `gh` does not support the operation or access
through the CLI is unavailable.

## Commits

When relevant task changes form a coherent completed unit, validate as
appropriate and automatically create a local git commit. Use a descriptive
message that explains what changed and why; include useful context in the body
when needed. Commit at meaningful completed milestones and skip commits when a
task creates no relevant changes. Stage only files or hunks owned by the task,
preserving unrelated and pre-existing work. Do not create empty commits or
automatically push. Report validation blockers honestly.
