package frc.robot.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.constants.types.PositionJointConstants.*;

/** Physical state never changes when the controller's user reference is zeroed. */
public final class JointPhysics {
  private final PositionJointHardwareConfig config;
  private final JointSimulationConfig physical;
  private final DCMotor motor;
  private final ElevatorSim elevator;
  private final SingleJointedArmSim arm;
  private final double phase;
  private double current;

  public JointPhysics(PositionJointHardwareConfig c, JointSimulationConfig p, DCMotor motor) {
    this.config = c;
    this.physical = p;
    this.motor = motor;
    phase = c.gravityType() == GravityType.SINE ? 0.25 : 0;
    if (c.mechanismType() == MechanismType.LINEAR) {
      elevator =
          new ElevatorSim(
              motor,
              c.gearRatio(),
              p.massKg(),
              c.outputRadiusMeters(),
              p.minPosition(),
              p.maxPosition(),
              true,
              p.initialPosition());
      arm = null;
    } else {
      elevator = null;
      arm =
          new SingleJointedArmSim(
              motor,
              c.gearRatio(),
              p.outputInertiaKgMetersSquared(),
              p.armLengthMeters(),
              (p.minPosition() - phase) * 2 * Math.PI,
              (p.maxPosition() - phase) * 2 * Math.PI,
              c.gravityType() != GravityType.CONSTANT,
              (p.initialPosition() - phase) * 2 * Math.PI);
    }
  }

  public double position() {
    return elevator != null
        ? elevator.getPositionMeters()
        : arm.getAngleRads() / (2 * Math.PI) + phase;
  }

  public double velocity() {
    return elevator != null
        ? elevator.getVelocityMetersPerSecond()
        : arm.getVelocityRadPerSec() / (2 * Math.PI);
  }

  public double current() {
    return current;
  }

  public void validateLimits(double min, double max) {
    physical.validateLimits(min, max);
  }

  public void step(double appliedVoltage, boolean brake, boolean driven) {
    boolean coast = !driven && !brake;
    double volts =
        coast
            ? velocity() * config.motorRotationsPerUnit() * 2 * Math.PI / motor.KvRadPerSecPerVolt
            : appliedVoltage;
    if (elevator != null) {
      elevator.setInputVoltage(volts);
      elevator.update(0.02);
      current = coast ? 0 : Math.abs(elevator.getCurrentDrawAmps());
    } else {
      arm.setInputVoltage(volts);
      arm.update(0.02);
      current = coast ? 0 : Math.abs(arm.getCurrentDrawAmps());
    }
    SimulationPower.report(this, current);
  }
}
