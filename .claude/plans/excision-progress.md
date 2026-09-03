# Excision progress

| task | status | commit | notes |
|---|---|---|---|
| 0.1 | done | 0aa2f0f79 | rewire Project.toml onto IOM and POM; deps resolve |
| 0.2 | done | ef3247e80 | add static undefined-symbol checker (removed in 5.2, scaffolding only) |
| 1.1 | done | 220e6d248 | remove model-building code moved to IOM and POM |
| 1.2 | done | 3bcb492d1 | rewrite main module onto IOM and POM |
| 1.3 | done | dc6ef5d57 | cut SPLIT files to simulation fragments and fence events |
| 1.4 | done | e5d0279d0 | module loads against IOM and POM |
| 2.0 | done | 4cd8a54a9 | close undefined symbols from the checker baseline |
| 3.1 | done | 2d6ffdb9e | smoke simulation builds, executes and reads results |
| 3.2 | done | c787550fb | fix undefined exports so Aqua passes |
| 4.1a | done | 74f52829e | rewire test environment onto IOM and POM |
| 4.1 | done | 7354027b6 | test infrastructure onto IOM and POM |
| 4.3 | done | b478f3bc2 | simulation build tests (34/34) |
| 4.4 | done | a5a04309a | market bid and import/export cost simulation tests |
| 4.5 | done | b509749b0 | simulation execute tests (46/46) |
| 4.6 | done | 9e703b273 | simulation store and results tests (15,278 assertions) |
| 4.7 | done | e8b9ec797 | partitioned simulation tests (441/441) |
| 4.7b | done | 650833f64 | fix initial-condition reconciliation and recorder registration |
| 4.8 | done | 1420b4486 | recorder (4/4), print (14/14), and util (2/2) tests |
| 5.1 | done | 50e5240a8 | prune docs to simulation scope; docs build clean |
| 5.2 | done | (this commit) | guides reflect PSI scope; remove excision tooling |

## Final summary

- 19 commits landed on `jd/pom_excision` before 5.2 (`git log --oneline main..HEAD`), plus the
  5.2 guide/cleanup commit.
- `git diff --shortstat main..HEAD`: 221 files changed, 3,847 insertions(+), 70,844 deletions(-);
  149 files deleted outright.
- `grep -rn EVENTS-EXCISION src test | wc -l`: 20 fenced blocks/includes — the events-to-POM
  follow-up (spec §8) is the complete list of what remains fenced.
- Module loads (`using PowerSimulations`), Aqua passes clean, docs build clean.
- Every simulation test file is green: partitions 441/441, execute 46/46, build 34/34,
  store+results+export 15,278 assertions, market-bid and import/export cost files fully green
  (with `UPSTREAM-IOM-BUG`-marked branches documenting the frozen-PWL-breakpoint bug, not
  PSI bugs), recorder events 4/4, print 14/14, sequence 12/12, models 3/3, utils 2/2.
- Two upstream bugs found and recorded in `.claude/CLAUDE.md` (not PSI's to fix): IOM's frozen
  PWL breakpoints in `add_pwl_block_offer_constraints!`, and POM standalone emulation not
  updating between executions.
- Follow-ups intentionally out of scope (spec, plan "Follow-ups" section, and `.claude/CLAUDE.md`):
  events framework port to POM then unfence; AGC; service feedforwards; `[sources]` path pins to
  git rev pins before a PR to `main`; PowerAnalytics/PowerGraphics downstream re-validation.
