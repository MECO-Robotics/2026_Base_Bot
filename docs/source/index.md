# 2026 Base Bot

This site documents the base FRC robot project in this repository and uses the
[Furo theme](https://github.com/pradyunsg/furo) for a clean, navigable reference.

```{toctree}
:maxdepth: 2
:caption: Docs

getting-started
architecture
subsystems
customizing
```

## What this project gives you

- A command-based WPILib robot project targeting Java 17 and GradleRIO 2026.
- A swerve drive stack with PathPlanner, AdvantageKit logging, and SysId hooks.
- Reusable IO-backed subsystem patterns for flywheels, joints, sensors, piece detection, and vision.
- Real, sim, and replay execution modes so development is not blocked on hardware.

## Read this first

If you are adopting this repo for a season robot, start with
{doc}`getting-started`, then move to {doc}`architecture` before creating new
subsystems or commands.
