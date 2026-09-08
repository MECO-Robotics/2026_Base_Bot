# Ninjineers 2026 Base Robot Code

FRC Robot Code has gotten significantly more advanced in recent years with brushless motors, external CAN sensors, vision, and more complicated odometry. Ninjineers has realized that these advancements are not possible over the course of a single season so we created this base code to allow teams to focus on the emergent behavior of their robot instead of getting bogged down in the weeds of motor control. In developing this template we tried to create sensible defaults for TalonFX's and Spark Max's for motor control, and integrated Mechanical Advantage's Drivetrain and Vision projects (with modification to enable more modular configuration). These defaults work for us but you are free to create your own [IO layers](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) or bespoke Subsystems. We would love it if you create a pull request to share your code with other teams.

## Getting Started

1. Create a fork of this repo, this will allow you to pull the latest changes in this repo directly from the Github UI.
2. Pull your fork
3. Split up your robot into different subsystems (Check out the [Subsystems](#subsystems) section)

## Documentation

This repository now includes a Sphinx documentation site under `docs/` using the
[Furo theme](https://github.com/pradyunsg/furo).

Hosted docs: https://meco-robotics.github.io/2026_Base_Bot/

Build it locally with:

```powershell
cd docs
py -m pip install -r requirements.txt
py -m sphinx -b html source build/html
```


## Subsystems

We split our robot into many small subsystems to allow for heavy reuse of subsystems (this is why we have only two motor control subsystems).

* Flywheel

  Flywheel is the more basic of the two subsystems and allows for controlling the velocity or voltage of a single motor or a group of motors. The biggest constraint with a Flywheel subsystem is it can only have one velocity setpoint or velocity measurement. This means that every motor will be spinning together (probably in the same gearbox).

* PositionJoint

  PositionJoints are more complicated as they can represent a Pivot or Elevator. A PositionJoint supports Position and Voltage control modes. Just like the Flywheel a single PositionJoint can also only have one position setpoint and position output but can have multiple motors.

### Splitting a Subsystem

Many of what we want to think of as a single robot Subsystems will need to be represented as multiple Subsystems in code. For example: a shooter from Crescendo might have a set of left wheels and a set of right wheels that spin at different velocities to create spin. This should be represented as two Flywheels (one for the right and one for left). The differential velocity is a more complicated emergent behavior that should be controlled at the command level not the subsystem level.

## Mechanism configuration and units

Angular mechanisms use output rotations and rotations/second. Linear joints use metres and
metres/second. `gearRatio` always means motor rotations per output-shaft rotation; a linear
joint converts through its drum circumference, `2 * Math.PI * outputRadiusMeters`.

Hardware mapping, encoder calibration, and simulation physics are separate records. Pass the
simulation record explicitly to `fromSparkMax` or `fromTalonFX`. Custom implementations use
`fromMode(name, realSupplier, simSupplier)`; replay calls neither supplier.

See the [mechanism configuration guide](docs/source/mechanism-contracts.md) for gain units,
encoder support, gravity phase, reset semantics, and complete downstream migration examples.
Example constants are illustrative and require measurement and tuning on your robot.

See [the Rebuilt migration notes](docs/rebuilt-migration.md) for preserved source history,
audit fixes, automated acceptance, and deferred Rebuilt ports.
