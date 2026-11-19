# Agent guide

- Start with [docs/README.md](docs/README.md); consult linked docs only for the area being changed.
- ROS commands run in Ubuntu 24.04 WSL from `/mnt/c/Users/pauso/Documents/GitHub/saprog`, sourcing `/opt/ros/jazzy/setup.bash`. Keep colcon build/install/log under `/home/<user>` and run dependent build/test steps in one WSL call. If WSL returns `E_ACCESSDENIED`, retry with approved elevation. See [testing](docs/testing.md).
- For desktop UI changes, follow [desktop style](docs/desktop-style.md). Before shared helpers or new module dependencies, check [reuse map](docs/reusability.md) and its linked owner. Keep one clear owner and preserve validation, cleanup and hardware safeguards. Update the reuse map when a shared API changes.
- Make every new user-facing tool available in Home's **Other panels** section, using a discoverable launcher in `tabs/`. Preserve each tool's hardware and process ownership requirements.
- Put proposed APIs in issues; do not document them as implemented. For substantial tasks, create an issue with acceptance criteria before implementation. Small, self-contained work needs no issue.
- Use the judge/developer workflow only when `$work` or `/skills` explicitly selects it.
- Use `gh` for supported GitHub work; run GitHub CLI commands with network escalation.
- At a coherent milestone, validate as appropriate and commit only task-owned files/hunks. Preserve unrelated work; do not push. Include an issue reference when applicable.
