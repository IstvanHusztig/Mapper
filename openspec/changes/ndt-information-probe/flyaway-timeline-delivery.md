# Offline synchronized timeline

User authorized building this view on Medium, explicitly keeping the replay
stopped. Implemented tools/flyaway_timeline.py using retained JSON, CSV and NumPy
samples. No new package, mapper run or numerical setting change.

Delivered to:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-motion-prior/timeline-v2/index.html`.

Standalone HTML embeds the selected data; no network/CDN or server is required.
Six diagnostic charts share a selected window with raw sensor-frame and final
world-frame cloud views. Slider, chart clicking, stepping, playback, projection
selection and event shortcuts update the same selection. Existing LAZ extracts
are linked relative to the artifact. Inputs and generated HTML are hashed in
manifest.json. Original artifacts remain unchanged.

Limits: scene data exist only at240–285s and300–320s, with explicit NOT MEASURED
elsewhere. Scene display is subsampled, uses a two-second interval, and clips to
the labelled view extent. Raw points have no deskew; world points inherit final
trajectory errors. World view is centred on displayed points, not sensor pose.
Endpoint measurements are labelled by their window's start with approximate end
shown separately. Displayed speed/heading triggers are existing diagnostics,
not a new validated flyaway detector. Detailed solver factors are unavailable
until the separately authorized replay is resumed and validated.

Verification: generated from1517 retained windows; input monotonicity and finite
sample checks passed. Node DOM/canvas stub smoke exercised render logic, seeking,
missing cloud coverage, projections, stepping, slider and chart click. Native
browser visual layout has not been independently verified. No accuracy verdict
or causal conclusion is added by this UI. Replay remains stopped. Medium is
sufficient for this tooling; notify user before High is needed for causal work.

The initial HTML failed a JavaScript escaping smoke check; the template was corrected and the successful build is timeline-v2. The original timeline directory is superseded.
