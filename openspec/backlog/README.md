# Backlog

Changes that are fully planned but not started. Each folder has the same files as a change in `openspec/changes/`
(`.openspec.yaml`, `proposal.md`, `design.md`, `tasks.md`, `specs/`), so it can be started without planning again.
The OpenSpec CLI only works on `openspec/changes/`, so a change here does not show up in `openspec list` and is not
validated.

**Starting a backlog change:**
1. `git mv openspec/backlog/<name> openspec/changes/<name>`
2. `openspec validate <name>`: fix whatever the CLI reports (the files were written without it).
3. Re-read `design.md` → "Open questions" and the "Before starting" task group: the code may have moved on since the
   plan was written.
4. Apply as usual (`/opsx:apply`).

**Adding to the backlog:** plan the change in `openspec/changes/` as usual, then `git mv` it here and add a line
below.

| Change | Planned | Summary | Effort |
|---|---|---|---|
| `modern-gui` | 2026-11-17 | GLFW instead of freeglut, a camera module, a shader renderer with edge shading (eye-dome lighting), a docked layout with Run and Log panels, dark and light themes; target layout in `modern-gui/mockup/target-layout.html` | about 3.5–4 weeks |
