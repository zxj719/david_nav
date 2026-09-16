# ADR-0011: collect idle evidence before changing the spin loop

Status: measurement tooling implemented (P6.5 R6); architecture decision deferred.

SPEC requires a 60-second idle sample before considering the 3%-of-one-core gate.
P6.5 authorizes only the measurement script. pidstat is not currently on the David
PATH, and installation is prohibited, so a standard-library Linux proc sampler
records equivalent process CPU counters plus raw timing and identity evidence.
It does not claim byte-for-byte pidstat output equivalence.

An explicit PID is mandatory; target discovery/start/stop, ROS and power tuning
are outside this tool. Window completeness, start-tick identity, monotonic counter
checks, cadence and operator-confirmed idle all gate the optional 3% comparison.
Partial, sparse, unconfirmed or interrupted captures cannot make that conclusion.
See [usage and artifact contract](idle-profiling.md) for exact fields and exit codes.

Fake counters/virtual time cover both sides of the threshold, >100% CPU, initial
baseline exclusion, PID reuse, counter/clock rollback and partial captures. Tiny
self-process probes check Linux I/O and installed entry points only. No real
navigation idle window has been collected or compared here; the 100 Hz runtime
and executor-thread architecture remain unchanged. Review an actual comparable
baseline before choosing either SPEC branch. No synthetic result substitutes for it.
