# Architecture

## Design approach

The base project is organized around two ideas:

- Commands orchestrate robot behavior.
- Subsystems hide hardware details behind IO interfaces.

That separation keeps the code portable across real hardware, simulation, and log replay.

## Top-level flow

`RobotContainer` is the assembly point for the robot. It:

- Instantiates real, sim, or replay IO implementations based on the active mode.
- Builds the `Drive` subsystem and wires PathPlanner autonomous support.
- Registers characterization and SysId routines with the dashboard chooser.
- Assigns controller bindings and default teleop behavior.

`Robot` then delegates lifecycle behavior to the command scheduler.

## IO layer pattern

Most reusable mechanisms follow an IO-backed model:

1. Define an IO interface for sensor and actuator access.
2. Implement that interface for each hardware target.
3. Keep subsystem logic independent from specific motor controllers.

Examples in this repository include:

- `DriveMotorIO*` and `AzimuthMotorIO*` for swerve modules.
- `FlywheelIO*` for shooter or intake-style wheels.
- `PositionJointIO*` for pivots and elevators.
- `VisionIO*` for Limelight, PhotonVision, simulation, and replay sources.

## Logging and tuning

AdvantageKit is built into the project and used throughout the subsystem stack:

- Inputs are logged via `Logger.processInputs(...)`.
- Tunable gains are exposed with `LoggedTunableNumber`.
- Replay mode allows validating code against recorded logs.

This is the main reason the project can support aggressive iteration without coupling behavior to hardware availability.

## Drive stack

The swerve implementation provides:

- A four-module `Drive` subsystem.
- Pose estimation via `SwerveDrivePoseEstimator`.
- PathPlanner integration through `AutoBuilder`.
- SysId and feedforward characterization hooks.
- Optional Phoenix and Spark odometry threads depending on motor vendor mix.

The drive code is already set up for live gain updates and module limit tuning through logged values.
