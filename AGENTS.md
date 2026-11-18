# Agent entry point

Start with [docs/README.md](docs/README.md) for architecture, behavior to preserve,
validation limits and open issues. Keep project context in [docs/](docs/README.md).
Before creating or changing any desktop tab, window, or UI, read and follow
[docs/desktop-style.md](docs/desktop-style.md).

Use the judge/developer workflow only when the user explicitly invokes `$work`
or selects it through `/skills`. Ordinary requests and follow-ups do not enable it.

## Commits

When relevant task changes form a coherent completed unit, validate as
appropriate and automatically create a local git commit. Use a descriptive
message that explains what changed and why; include useful context in the body
when needed. Commit at meaningful completed milestones and skip commits when a
task creates no relevant changes. Stage only files or hunks owned by the task,
preserving unrelated and pre-existing work. Do not create empty commits or
automatically push. Report validation blockers honestly.
