# Subsystems

## Drive

The drive base is the most complete example in the project and acts as the template for larger mechanisms.

Key responsibilities:

- Read gyro and module state inputs.
- Maintain odometry and fused pose estimation.
- Convert chassis requests into per-module swerve setpoints.
- Expose autonomous, characterization, and teleop-friendly command entry points.

## Flywheel

`Flywheel` is the simplest reusable motor subsystem. It is appropriate when one logical mechanism shares a single velocity target.

Use it for:

- Shooter wheels spinning together.
- Conveyor rollers.
- Intake rollers with a shared speed goal.

It supports:

- Velocity control
- Direct voltage control
- Tunable feedforward and feedback gains
- Motion Magic/MAXMotion acceleration constraints handled by vendor IO

## PositionJoint

`PositionJoint` covers mechanisms that need bounded positional control, including pivots and elevators.

It supports:

- Position targets with tunable constraints
- Optional dynamic velocity overrides
- Direct voltage control
- Soft range limits
- Gravity-aware feedforward

This is the subsystem you should reach for when the mechanism is fundamentally defined by where it should be, not just how fast it should spin.

Joint position commands finish at tolerance and retain their goal afterward. An optional `complianceAfterTarget` argument releases the motor into coast at the target until a new goal is requested. Compliance is disabled by default. Direct voltage commands retain output ownership until the next position request.

Flywheel and joint simulation implementations now live beside their hardware IO in `subsystems/flywheel` and `subsystems/position_joint`. Their `fromSparkMax` and `fromTalonFX` factories select real, simulated, or replay IO. Hardware configuration, encoder calibration, and physical simulation parameters are separate. See {doc}`mechanism-contracts` for the required factory arguments, units, gravity, and reset semantics.

## Vision

The vision stack accepts one or more camera implementations and filters observations before passing them into a consumer, usually the drive pose estimator.

Important behavior:

- Rejects invalid or out-of-bounds observations.
- Scales measurement uncertainty from ambiguity, tag count, and target distance.
- Logs accepted and rejected estimates for debugging.

`VisionIO` provides mode-aware Limelight and QuestNav/PhotonVision factories, including simulation adapters. `VisionConstants` retains the template camera transforms. Tag filtering defaults to one tag with ambiguity checks and an empty whitelist; configure `odometryTagWhitelist` and minimum counts for your robot. QuestNav inertial observations do not require visible tags. These adapters are available for wiring into `RobotContainer`; the template does not instantiate a camera automatically.

## Sensors and game-piece detection

The repository also includes smaller building blocks for:

- Digital sensors
- Game-piece detection
- Absolute encoders
- PathPlanner utilities

Those classes are intentionally narrow. They are meant to be composed into higher-level robot behaviors rather than expanded into monolithic subsystems.

`UnitInterpolatingMap` interpolates unit-aware measurements, clamps queries to the supplied range, and returns zero in the value unit when empty.
