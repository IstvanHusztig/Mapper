## ADDED Requirements

### Requirement: A passage walked out and back is judged without a reference length
A tool SHALL read a result of a recording in which a passage was walked out and back and SHALL report, along the
passage's main direction: the turning point and how far the way back departs from the mirror image of the way out; the
travel per window with its spread and the number of windows that stand still or go back between the first and the last
moving window; and how well the surfaces across the passage seen on the way out coincide with those seen on the way
back (their agreement at no shift, and the shift at which they agree best). It SHALL also report the passage's length
in the result and the distance between the start and the end, as readings. A reading it cannot compute SHALL be NOT
MEASURED. The tool SHALL state that these are agreement within one recording and assumptions about the walk (out and
back along the same passage, at a steady pace), not accuracy.

#### Scenario: A passage folded onto itself
- **WHEN** the result places a passage walked out once and back once as several short forward and backward stretches
- **THEN** the tool reports windows that go back before the turning point and a large departure from the mirror image

#### Scenario: Out and back agree
- **WHEN** the result places the way back along the way out, with the door frames of both ways at the same places
- **THEN** the surfaces across the passage agree best at a shift within one bin, and the tool says so with the agreement's value

#### Scenario: No surfaces across the passage
- **WHEN** neither way has a surface across the passage
- **THEN** the agreement of the surfaces is NOT MEASURED and the other readings are still given
