# 2026 Base Bot

This site documents the base FRC robot project in this repository and uses the
[Furo theme](https://github.com/pradyunsg/furo) for a clean, navigable reference.

If you are new to the codebase, start with {doc}`getting-started`. It is written
as a first-pass adoption guide, not just an API reference.

```{toctree}
:maxdepth: 2
:caption: Start Here

getting-started
```

```{toctree}
:maxdepth: 2
:caption: Advanced

architecture
subsystems
mechanism-contracts
customizing
```

## What this project gives you

- A command-based WPILib robot project targeting Java 17 and GradleRIO 2026.
- A swerve drive stack with PathPlanner, AdvantageKit logging, and SysId hooks.
- Reusable IO-backed subsystem patterns for flywheels, joints, sensors, piece detection, and vision.
- Real, sim, and replay execution modes so development is not blocked on hardware.

## Student workflow

For most students, the normal job is:

1. Configure constants for the real robot.
2. Assemble command compositions and controller bindings in `RobotContainer`.
3. Tune and test.

You usually do not need to rewrite subsystem internals or IO layers unless the
robot has a genuinely new hardware requirement that the base patterns do not cover.

## Start here

- Read {doc}`getting-started` first.
- Make your early changes in `frc.robot.constants` and `RobotContainer`.
- Open the Advanced section only when you need deeper implementation detail.

## Advanced reference

- {doc}`architecture`
- {doc}`subsystems`
- {doc}`customizing`
