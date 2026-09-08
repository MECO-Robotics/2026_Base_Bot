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

Validation: `JAVA_HOME=/home/brian/wpilib/2026/jdk bash gradlew build --console=plain` passed, including formatting, compilation, packaging, and four regression tests covering goal retention/open-loop control, compliance, vision filtering, and unit conversion/interpolation. Use your WPILib JDK installation path; this project's existing formatter fails under the workstation's default JDK 21. Existing drivetrain vendor API deprecation warnings remain. No physical robot or camera validation was performed.

The original Base Bot worktree and its uncommitted changes were left intact. This migration does not include those local edits.
