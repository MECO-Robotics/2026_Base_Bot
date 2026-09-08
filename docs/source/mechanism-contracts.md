# Mechanism contracts and migration

## Units and configuration

All angular goals, feedback, tolerances and limits use **output rotations**. Angular rates
use rotations/second. Linear joints use **metres** and metres/second. Profile acceleration
uses the corresponding units/second². Gearing is motor rotations per output-shaft rotation:

- Angular motor rotations = output rotations × gearing.
- Linear motor rotations = metres × gearing / (2π × drum radius in metres).
- Physics converts output rotations to radians only at the WPILib plant boundary.

`FlywheelHardwareConfig` contains CAN IDs, directions, gearing, current limit and bus.
`PositionJointHardwareConfig` additionally selects mechanism type, independent gravity type,
drum radius, and an `EncoderCalibration`. The first direction flag inverts the leader;
follower flags specify opposition relative to that leader. Followers receive current limits
before following. Motor arrays must be nonempty, equal length, and contain unique valid CAN IDs.
Gearing, current limits, and required physical parameters must be positive and finite.

`FlywheelSimulationConfig` specifies output-side inertia in kg·m². `JointSimulationConfig`
specifies output-side inertia, elevator mass, arm length, immutable physical minimum/maximum,
and initial position. Supply positive values for all three physical dimensions; angular plants
use inertia and arm length, and linear plants use mass and the hardware drum radius. Software
limits may be tuned within physical travel, but cannot enlarge the plant or exceed its bounds.
Linear mechanisms require a positive drum radius and CONSTANT gravity.

## Gains and gravity

Let U mean one output rotation for angular mechanisms or one metre for linear mechanisms.
Use voltage-based gains for both vendors; Spark PID is converted internally to its duty-cycle
and millisecond convention with nominal 12 V compensation.

| Gain | Position joint | Flywheel velocity loop |
| --- | --- | --- |
| kP | V/U | V/(U/s) |
| kI | V/(U·s) | V/U |
| kD | V/(U/s) | V/(U/s²) |
| kS, kG | V | kS in V; no gravity term |
| kV | V/(U/s) | V/(U/s) |
| kA | V/(U/s²) | V/(U/s²) |

`GravityType.COSINE` defines zero horizontal. `SINE` defines zero vertical, with positive
quarter rotation horizontal. CONSTANT supplies a fixed gravity voltage. Native CONSTANT and
COSINE terms are enabled on both vendors. SINE is software feedforward and disables native
gravity; external voltage feedforward is added exactly once on normal and constrained requests.
Use calibrated physical position for gravity, independent of runtime reset.

Do not copy the old radian-based gearing, limits or tuning values. Convert angular positions
and velocities by dividing radians by 2π. Gains expressed per radian become gains per rotation
by multiplying by 2π (including derivative and integral gains after confirming their original
time convention). Measure actual gearing, mass, inertia, encoder phase and gravity voltage.
The supplied EXAMPLE constants are illustrative, not validated robot tuning.

## Sensors, control and reset

`EncoderCalibration(type, id, unitsPerRotation, offsetRotations, reversed)` describes the
external sensor independently of motor gearing. The calibrated coordinate is
`(directedSensorRotations + offsetRotations) * unitsPerRotation`, with absolute sensors wrapping
at their sensor revolution boundary. Choose calibration and travel to avoid crossing that
boundary for a single-turn Spark absolute sensor. Internal feedback uses
`EncoderCalibration.internal()`; nondefault internal calibration is rejected.

Spark joints support brushless INTERNAL and EXTERNAL_SPARK feedback. Talon joints support
INTERNAL, EXTERNAL_CANCODER and EXTERNAL_CANCODER_PRO (fused feedback requires the vendor
license). DIO boot-seeding and mismatched vendor sensors are rejected explicitly instead of
silently using a different feedback source. Controller feedback, completion, compliance and
soft limits all use the selected calibrated mechanism coordinate.

Reset changes the reported sensor reference without teleporting the mechanism. It zeros the
reported position and subsystem goal, preserves physical velocity and calibration, clears
temporary constraints/compliance, and exits voltage control. Requests are translated back
through the runtime offset; software limits remain anchored to their calibrated physical
coordinate. Reset is not physical homing and does not redefine gravity orientation.

Position joints hold their goal after completion. Temporary profile constraints are restored
on the next ordinary request after clearing or command interruption. Compliance remains
released until a new goal. Flywheel goals belong to commands (including defaults), persist
after successful completion, and stop on interruption. Dashboard edits apply only when no
command owns the subsystem; ignored edits do not take effect merely because a command ends.
Voltage-command cancellation and SysId completion explicitly send zero volts.

## Downstream examples and API breaks

The old constructors that combined hardware, encoder offset and simulation inertia are removed.
The IO position API is `setPosition(position)`; use `setPositionDynamic(position, maxVelocity,
maxAcceleration)` for explicit profile constraints. There is no unused velocity argument.

```java
import static frc.robot.constants.types.PositionJointConstants.*;

var hardware = new PositionJointHardwareConfig(
    new int[] {10, 11}, new boolean[] {false, true},
    12.0, 35, "", MechanismType.LINEAR, GravityType.CONSTANT,
    0.025, EncoderCalibration.internal());
var physics = new JointSimulationConfig(0.1, 6.0, 0.5, 0.0, 2.2, 0.2);
// Illustrative gains only; tune the real elevator before enabling it.
var gains = new PositionJointGains(
    4, 0, 0, 0, 0, 0, 0, 0.5, 1, 0, 2.1, 0.01, 0.2);
var elevator = new PositionJoint(
    PositionJointIO.fromTalonFX("Elevator", hardware, physics), gains);
```

```java
var wheelHardware = new FlywheelHardwareConfig(
    new int[] {15}, new boolean[] {false}, 2, 30, "");
var wheelPhysics = new FlywheelSimulationConfig(0.025);
var io = FlywheelIO.fromSparkMax("Wheel", wheelHardware, wheelPhysics);
// Custom mode selection must supply a functioning simulation explicitly:
var custom = FlywheelIO.fromMode("Wheel",
    () -> new FlywheelIOSparkMax("Wheel", wheelHardware),
    () -> new FlywheelIOSimSparkMax("Wheel", wheelHardware,
        wheelPhysics, DCMotor.getNEO(1)));
```

## Simulation and telemetry

Joint simulations share the real vendor control implementation. Disabled controllers supply
no drive voltage while physics continues evolving. Neutral brake uses motor electrical damping;
coast removes that electrical load, rather than artificially freezing a stationary mechanism.
Native controller simulators retain vendor-specific timing/current-limiter behavior.

`Robot.robotPeriodic` starts one `SimulationPower` cycle before scheduler updates and computes
battery voltage once afterward for the next loop. Every plant, including drivetrain plants,
reports aggregate current once. A repeated report replaces that plant's current rather than
adding it twice. Multi-motor telemetry divides aggregate plant current among motors. Custom
simulation loops must call `beginCycle`, update all mechanisms, then `endCycle` once.

Joint `outputPosition`, `velocity` and `desiredPosition` use mechanism units. Joint motor
position/velocity telemetry uses rotor rotations/second, independently of selected external
feedback. `motorVoltages` is applied motor voltage, not supply voltage. `motorSupplyVoltages` separately identifies measured supply. Connection alerts assert on
failure, not on successful communication.

## Vision and replay

Every `PoseObservation` now requires an immutable set of IDs from that individual solve.
Camera batch IDs are visualization only. A nonempty whitelist with a positive minimum requires
that many **distinct permitted IDs in each observation**. Missing metadata rejects the
observation when that filter is enabled. An empty whitelist disables this restriction.
Vision fusion and QuestNav absolute correction share the same finite-value, ambiguity, bounds
and ID validation. QuestNav inertial poses are exempt from visible-tag requirements.

`VisionIOInputsLogged` retains the old pose struct schema and records per-observation IDs in
separate `ObservationTags/<index>` arrays. Old replay records load empty per-observation sets
and remain usable with whitelist enforcement disabled. Producers must supply IDs when creating
observations; do not substitute the camera's batch union. Limelight coordinate conversion is
unchanged.
