## MODIFIED Requirements

### Requirement: Run-dependent fields are normalized
The comparison SHALL ignore only these run-dependent values and nothing else:
the date-time part of the `dMapper_params_*` and `dMapper_results_*` file names;
elapsed-time values (`elapsed time seconds`, `elapsed_time_seconds`); `processing_date`;
`build_date`; and absolute paths, which are compared after replacing the output-folder and
input-folder prefixes with placeholders.

#### Scenario: Only timing differs
- **WHEN** two output folders differ only in elapsed time, processing date, build date, result file name timestamps and the output folder location
- **THEN** the comparison reports them as identical

#### Scenario: A trajectory value differs
- **WHEN** one value in any `trajectory_lio_*.csv` differs in its last digit
- **THEN** the comparison reports that file as different
