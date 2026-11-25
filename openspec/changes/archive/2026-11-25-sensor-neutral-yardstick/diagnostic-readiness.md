# Diagnostic replay and restricted use

Superseded for current tool behavior by tool-validation.md after the user-approved
implementation. The examples below document the pre-fix behavior and motivation.

2026-11-12. No application or diagnostic-tool code changed. Reproducible local
probe: out/phase0a-2026-11-12/check_diagnostics.py, invoked with python -B.
Raw report: diagnostic-coverage.json in that same output directory.

## Known-log replay

| Log | Windows | Heading / speed / bias coverage | Weak-axis chi-square coverage | Bends >3 / >10 degrees |
|---|---:|---|---|---|
| turn baseline | 197 | 197 each | 0: NOT MEASURED | 1 / 0 |
| Livox1 historical shipped base | 261 | 261 each | 52/261, conditional subset | 16 / 3 |
| Livox2 historical shipped base | 1517 | 1517 each | 616/1517, conditional subset | 168 / 46 |

The replay independently counted window IDs and heading threshold crossings,
then asserted agreement with run_health.analyse. Livox1's actual logged count
is 261; the older draft's 260 is not authoritative. These are diagnostics of
existing logs, not a candidate ranking or an assertion of hardware failure.
The historical files are identified by exact paths in the JSON report.

## Failure examples

- Removed all optional diagnostics from a copy of the turn log while preserving
  window-start lines. run_health CLI exited 0 and printed PASS. Heading, speed,
  bias and chi-square are actually NOT MEASURED. See missing-diagnostics.log
  and missing-diagnostics-tool-output.txt in the output directory.
- Synthetic three-row trajectory versus its two-row prefix: seam compare
  returned zero displacement over two rows, silently truncating coverage.
- Same synthetic positions with every timestamp shifted by 0.1 s: unchecked
  compare returned zero displacement. Exact count/timestamp checks rejected both
  examples as NOT COMPARABLE before any metric could support a verdict.

## Restricted procedure for the current documentation/verification scope

1. Do not use generic perturbation_sweep.py to launch/resume jobs or decide a
   verdict. It lacks provenance checks, aggregates legacy verdicts, does not
   propagate all failure conditions and can delete clouds. Keep existing files.
2. Use run_health.analyse only to extract identified log readings, with an
   independent per-window coverage inventory. Never promote CLI exit 0 or PASS
   into evidence of complete measurement. Do not use inferred stop reasons or
   approximate bend sigma. Missing loop-summary fields mean NOT MEASURED unless
   the original log separately establishes the stage outcome.
3. Before any seam calculation, verify same input/config family/stage/frame,
   equal counts and exact timestamp correspondence. Reject failed checks;
   never truncate, silently align, or scrape a warned CLI result. The two
   failure examples above verify count and timestamp rejection.
4. Require recorded input, configuration and executable identity plus successful
   completion before current acceptance comparisons. An absent identity is
   NOT ESTABLISHED; inventing or backdating a hash is prohibited. A current hash
   fingerprints the artifact now, not the executable that produced it earlier.
5. Historical evidence with provenance gaps may be inventoried and diagnostics
   replayed, but cannot certify today's configuration against accepted targets.
   Map coverage, all family members and both processing stages remain necessary.

This restriction supports raw evidence review only. An unattended comparison
workflow still needs separately scoped fixes or a fully verified manual run
procedure. Task 2.5 remains open for that complete workflow; do not mark it done
merely because the bad aggregate verdict is no longer used.
