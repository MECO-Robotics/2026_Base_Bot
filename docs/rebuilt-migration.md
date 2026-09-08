# Rebuilt generic-code migration

Compared Base Bot `36d35b5` with Rebuilt `f5004eebdb42ed146c1aa3d6a2d5a1052cd9371b` (also the fetched Rebuilt origin/main on 2026-09-08). Their common ancestor is `4a248a5`; Base Bot already included the earlier migration `438e740`.

| Area | Result |
| --- | --- |
| Flywheel and position joints | Imported reusable hardware IO, mode factories, simulation inertia/configuration, vendor motion control, commands, and SysId support. |
| Joint fixes | Cherry-picked `76a3c80` with `-x`; applied the subsystem/command portions of `0cff6e8` and `4dc3cc4` with original authorship and source references. Robot-specific portions were excluded. |
| Vision | Imported Limelight selection/simulation and QuestNav frame/simulation adapters. Retained Base Bot transforms, ambiguity filtering, and generic tag defaults. Camera wiring remains an application choice. |
| Utilities | Imported unit-aware interpolation and updated SysId statistics/publication. Added a compatible CANBus encoder overload. Other utility differences were primarily formatting and removed license headers, which were not copied wholesale. |
| Dependencies | Matched Rebuilt AdvantageKit, Phoenix 6, REVLib, PhotonVision, and QuestNav versions. Retained Studica because Base Bot still supports NavX. |
| Drivetrain | Retained multi-vendor drive/module/gyro IO. Rebuilt's CTRE-generated drivetrain, Choreo integration, carpet calibration, and robot gains are specific to its architecture. Base Bot already has logged drivetrain PID tunables and a SysId chooser. |
| Sensors | Existing digital-sensor and piece-detection behavior retained; upstream changes are predominantly formatting. |
| Competition code | Excluded shooter/intake/climber coordination, field scoring/timing helpers, robot visualization, fuel simulation, CAN mappings, autonomous paths, and controller bindings. |
| Project infrastructure | Retained Base Bot formatter, docs site, generated BuildConstants convention, and deployment settings. |

The generic mechanism baseline comes from Rebuilt `a5c137ae318a8ab756732a9a45ae4d5701cb1541`, immediately before `76a3c80`. Local branch `sources/rebuilt-main` retains the complete original source history for inspection and future ports. The migration is on `migrate/rebuilt-generic`; no synthetic merge marks the excluded competition changes as integrated.

Integration fixes make tunable checks sample every value, retain open-loop joint output until a position request, and keep compliance released after displacement until a new goal. Existing third-party notices are retained. Mechanism simulation classes now live next to their hardware IO.

## Audit remediation

The migration commits through `9c174b0` are retained unchanged. Focused fixes append explicit
configuration/units (`3a0afa5`), control and feedback (`8c10f1a`), simulation (`a6e5355`), and
per-observation vision validation (`7be4ca5`), followed by regression tests and documentation.
See [the mechanism guide](source/mechanism-contracts.md) for clean API breaks and adoption examples.

| Audit finding / confirmed limitation | Resolution and regression evidence |
| --- | --- |
| Flywheel target overwritten by idle/default dashboard values | Requested target is independent of telemetry; scheduler tests cover default, active, successful, interrupted, idle dashboard and stale IO cases. |
| Spark absolute control disagreed with telemetry/completion | Selected feedback supplies mechanism telemetry; native absolute feedback, reset, physical soft limits and rotor telemetry are exercised. |
| Spark dropped kV and external feedforward | Built-in kV is configured and external voltage reaches normal/dynamic requests; native configuration and output tests verify it once. |
| SINE gravity could not be selected | Independent gravity enum; analytic quarter-turn phase tests and vendor gravity slot readback. |
| Batch-wide whitelist authorized unrelated frames | Immutable per-solve IDs and one shared Vision/QuestNav correction validator; mixed frames, duplicates, malformed values and missing metadata tested. |
| Elevator simulation limited to ±1 metre | Explicit immutable physical travel, analytic linear gearing, >1 m initialization and rejection of out-of-range software limits. |
| Talon simulation removed gravity compensation | Sim inherits real Talon configuration/control, including native CONSTANT/COSINE and software SINE. Gravity slot readback and gravity-plant tests cover this contract. |
| Temporary profile limits persisted | Shared defaults remain separate from applied limits; both native controllers and command interruption restore ordinary-request defaults. |
| Reset did nothing or moved the simulation | Runtime reference offset, preserved physical position/velocity, goal zero, cleared compliance/overrides and voltage mode; hardware and simulation tests. |
| Inert generic factories | Explicit real/sim suppliers; replay constructs neither. |
| Followers, alerts and voltage telemetry | Correct leader CAN IDs, follower current limits, disconnect polarity, applied-voltage signals and separate supply-voltage telemetry. Native follower/configuration and voltage readback tests. |
| Artificial brake lock, disabled motion and shared battery | Electrical brake/coast model, live plant while disabled, normalized inversion, per-plant aggregate reporting and one battery update after scheduler. Both vendor simulations, multiple motors and order-independent battery tests. |

The ten original audit reproductions are replaced by maintainable configuration, scheduler,
serialization and vendor-simulation tests rather than reflection into implementation fields.
The original four migration tests remain, adapted only to the new public contracts. The suite
contains 29 tests. Full automated acceptance commands are:

```bash
JAVA_HOME=/home/brian/wpilib/2026/jdk bash gradlew build spotlessCheck --console=plain
python -m sphinx -W -b html docs/source docs/build/html
```

Install `docs/requirements.txt` in a virtual environment for the documentation command. Use your
WPILib JDK path; the existing formatter fails under the workstation's default JDK 21. The full
build includes compilation, packaging, formatting and all regression tests. Existing drivetrain
vendor deprecation warnings remain. GitHub Build and docs checks must pass on the final PR head.

Automated acceptance does not establish physical motor direction, encoder wrap/offset, native
firmware behavior, robot tuning, real battery characterization, or camera calibration/latency.
Validate those on the actual robot/cameras before deployment. No physical validation was performed.

## Deferred Rebuilt ports

Rebuilt remains untouched at `f5004eebdb42ed146c1aa3d6a2d5a1052cd9371b`. These are follow-up
ports, not instructions to cherry-pick Base Bot's template configuration into a competition robot.
Use the source commits below to review generic behavior, adapt callers and retest on Rebuilt:

| Base Bot source commit | Rebuilt follow-up |
| --- | --- |
| [3a0afa5](https://github.com/MECO-Robotics/2026_Base_Bot/commit/3a0afa5) | Split hardware/calibration/physics; migrate each angular caller from radians to rotations and each linear caller to metres. Audit every gearing and gain value before adoption. |
| [8c10f1a](https://github.com/MECO-Robotics/2026_Base_Bot/commit/8c10f1a) | Port target ownership, selected encoder feedback, complete feedforward, default constraint restoration, reference-only reset, follower limits and applied-voltage/connection telemetry. Adapt competition subsystem and command callers. |
| [a6e5355](https://github.com/MECO-Robotics/2026_Base_Bot/commit/a6e5355) | Share hardware/vendor-sim controls, configure real physical travel, use electrical neutral behavior and one aggregate simulation battery coordinator. Integrate Rebuilt's CTRE-generated drivetrain with the coordinator separately. |
| [7be4ca5](https://github.com/MECO-Robotics/2026_Base_Bot/commit/7be4ca5) | Add per-observation IDs across producers/logging; share finite-value and whitelist validation with QuestNav absolute correction. Preserve Rebuilt camera transforms and intentional filtering policy. |

Also port [ee5f9f5](https://github.com/MECO-Robotics/2026_Base_Bot/commit/ee5f9f5),
the subsequent Spark simulation refinement: use one motor model and per-controller
current for the native limiter while the physical plant and power coordinator retain aggregate
current. Port the accompanying regression suite along with those behaviors. Defer competition CAN mappings,
mechanism limits/gains, transforms, controller bindings, autonomous configuration and tuning changes
until a Rebuilt-specific review and physical validation. Limelight coordinate conversion is retained.

## Integration state

The original Base Bot checkout and user stash remain intact; these fixes use the migration worktree.
PR #8 must be merged with **rebase merge** to preserve individual commits under linear-history rules.
The repository's Protected Main update restriction currently prevents merging. An administrator must
resolve that restriction; this work does not change branch protections or rewrite migration history.
