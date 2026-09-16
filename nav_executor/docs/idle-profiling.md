# R6: passive idle CPU evidence

`nav_executor_profile` (or `python3 -m nav_executor.app.idle_profile`) samples one
explicit, already-running Linux PID. It never launches a target, sends a signal,
initializes ROS, connects to WS, changes priorities/power settings, or reads argv
or environment. Names embedded in stat are not recorded. No pidstat/psutil
installation is required. It uses
Linux `/proc/PID/stat` utime+stime, excluding child processes and including threads.

After an operator has selected the correct executor PID and confirmed it stays
idle for the whole window, use the refactor environment and a **new** output path:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws_refactor/install/setup.bash
python3 -m nav_executor.app.idle_profile --pid 12345 --seconds 60 --interval 1 \
  --confirm-idle --output ./executor-idle-baseline
```

Replace 12345 with the inspected target PID. This is not a command to start `3`,
navigation or robot motion. The source commands select Python packages only; the
sampler is not a ROS diagnostic and does not join DDS. Choose comparable baseline
and refactor conditions manually; do not label a busy/mixed run as idle. Without
`--confirm-idle`, the tool still records diagnostic data but makes no R6 conclusion.
The flag records the operator's assertion, not independent motion-state proof.

## Artifacts and validity

- `metadata.json`: PID, requested duration/interval, UTC start, boot ID, clock tick
  frequency, method/units and idle assertion. It contains no credentials or argv.
- `samples.csv`: baseline plus timed samples, raw CPU ticks, monotonic elapsed time,
  interval CPU percent, RSS bytes, thread count and process state. The baseline's
  CPU-percent field is blank, **not zero**. Rows are flushed after each sample.
- `summary.json`: total CPU delta divided by total elapsed time (time-weighted,
  not an unweighted mean), process start ticks, completeness and R6 eligibility.
  CPU 100% means one core; multithreaded processes can exceed 100%.

Exit 0 means a complete requested window, **not necessarily R6-eligible**. R6 also
requires requested duration >=60 seconds, interval <=1 second, the expected sample
count, no gap exceeding twice the requested interval, and confirmed idle. Only then
`r6_at_or_above_three_percent` is a boolean; otherwise it is null. The 3% comparison
is measurement evidence for review, not an automatic architecture change or a claim
of statistically significant improvement. Kernel tick quantization still applies.

PID reuse (start ticks changed), disappearing/zombie process, denied/malformed
counters, clock/counter regression or interruption invalidates the window. A partial
mean may remain for diagnosis but cannot support R6. Exit 1 means incomplete, 2 means
bad arguments/output, and 130 means interrupted. An absent/truncated final summary
after SIGKILL, reboot or storage failure is also incomplete; flushed CSV is not a
power-loss durability guarantee. Existing files/directories/symlinks are refused,
never overwritten or resumed. Parent directories must already exist.

## Decision status

P6.5 requests the measurement script only. This implementation and its virtual-time
tests are complete; no navigation idle baseline has been measured in this work.
The real 60-second comparison and any resulting spin-loop decision remain a manual
review item. The existing 100 Hz runtime is unchanged. Tiny self-process probes
verify packaging/read-only I/O only and are never reported as robot CPU evidence.
