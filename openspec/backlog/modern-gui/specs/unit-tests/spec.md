## ADDED Requirements

### Requirement: Camera tested without a display
The GUI's camera SHALL be built without OpenGL, GLFW or ImGui and tested in `tests/` without a display: the view matrix of each fixed view, orbit and pan steps, the orthographic projection, the smooth transition's end state, and picking the nearest trajectory node.

#### Scenario: Fixed views match the old viewer
- **WHEN** the camera tests run
- **THEN** each fixed view gives the view matrix the old viewer's `glRotate`/`glTranslate` sequence gave for it, to 1e-6
