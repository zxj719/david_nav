# POI data port migration

`nav_tasks.nav_site_data` is the logical `nav_site_data` boundary, physically
hosted in nav_tasks as expressly permitted by PLAN section 5.3. It imports no
ROS, message packages, transport or ROS adapters; AST tests enforce that rule.
This avoids a circular physical dependency between task-domain values and the
runtime composition root. It can be separated into a package when domain types
have an independent package; no additional process is introduced.

`YamlPoiRepository` implements the pure `PoiRepository` port. Its constructor
requires an explicit `map_frame`, used for reprojection output and the legacy
fallback when an input record omits its frame. All stored explicit frame labels
are retained. This fallback also applies to sparse odom records, matching the
old loader rather than silently reinterpreting existing point files.

The source YAML is read before the executor-owned file; local IDs win, ordered
dict insertion semantics stay the same. Loading skips the same unsupported
items and retains old conversion exceptions. `get` returns None for missing
IDs as required by the new port; skill decoding still produces the frozen
missing-POI error. Upsert, rename, session pruning and reprojection preserve
field order, Unicode, pose fields and PyYAML options. Differential tests compare
bytes after every operation against the unchanged legacy store.

## Compatibility decisions and limitations

- Reprojection retains raw `transform.yaw + pose.yaw` and raw angular threshold
  differences. The normalized general geometry helper is deliberately not used:
  wrapping would change persisted values and which records are rewritten near
  pi. Output frame is configured, not hardcoded. Height/stamp and odom anchors stay intact.
- A sibling `.tmp` is closed before `os.replace`, preserving the last complete
  destination on serialization/replacement failure. The repository remains a
  single-writer component, protected by the eventual runtime's ownership/lock;
  it is not safe for independent concurrent writers. No new fsync durability
  guarantee is claimed. File permissions and symlink behavior remain legacy.
- Mutations happen in memory before write. A failed write can leave memory ahead
  of disk and a partial temporary file, as before. Fault-injection tests make this
  explicit; transactional rollback is not silently bundled into migration.
- `import_named_poses` only reads the manual JSON source. It accepts the old list,
  `poses` and `named_poses` forms, including truthy fallback, skipped non-mappings,
  duplicate-ID counts and conversion behavior. Writes remain per-record. After
  successful iteration it prunes only the `named_poses` session and emits the
  frozen stale-ID message via an injected log sink. A later malformed item leaves
  earlier writes, and pruning is not performed after failure.
- Manual coordinates are copied into map/odom representations just as before;
  supplying `map_frame` and `odom_frame` only replaces old defaults. No manual
  point geometry or source file is rewritten. Source-frame semantics need the
  existing separate human review, not a covert migration fix.

The new protocol/TaskManager/Nav2Client chain is tested with this actual YAML
repository for mark, reload, rename and navigation; only motion/localization
are fakes and all files are under temporary test directories. No real POI files
or robot goals are touched.

The refactor runtime now assembles this repository, importer, TF pose source,
costmap observations and supervised loop-closure polling. Its ordinary POI/Dock
host and CLI are connected, and retained executable paths forward to the new
runtime; production aliases have not been switched. See [runtime.md](runtime.md)
and the fake-I/O factory tests in nav_executor. This is implementation evidence,
not a claim of complete real-action or physical acceptance.

Map YAML loading/trinary checks and recording hashes also exist in bringup tools;
adding this POI repository does not claim every map-data responsibility has moved
into a separate physical package. The constructor does not check whether the
source and cache paths alias: callers must choose a distinct writable cache.
Reload clears memory before merging files and can leave partially loaded records
after an exception. These are legacy boundaries, not transaction guarantees.
