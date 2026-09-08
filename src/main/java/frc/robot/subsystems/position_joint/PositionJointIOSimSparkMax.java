package frc.robot.subsystems.position_joint;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import frc.robot.constants.types.PositionJointConstants.*;
import frc.robot.sim.JointPhysics;

/** Real Spark control and calibration backed by a physical plant. */
public class PositionJointIOSimSparkMax extends PositionJointIOSparkMax {
  private final JointPhysics physics;
  private final SparkMaxSim[] simulations;

  public PositionJointIOSimSparkMax(
      String name,
      PositionJointHardwareConfig config,
      JointSimulationConfig simulation,
      DCMotor motor) {
    super(name, config);
    physics = new JointPhysics(config, simulation, motor);
    simulations = new SparkMaxSim[motors.length];
    for (int i = 0; i < motors.length; i++)
      simulations[i] =
          new SparkMaxSim(
              motors[i],
              new DCMotor(
                  motor.nominalVoltageVolts,
                  motor.stallTorqueNewtonMeters / motors.length,
                  motor.stallCurrentAmps / motors.length,
                  motor.freeCurrentAmps / motors.length,
                  motor.freeSpeedRadPerSec,
                  1));
    sync();
  }

  private void sync() {
    for (int i = 0; i < simulations.length; i++) {
      double direction = i > 0 && config.reversed()[i] ? -1 : 1;
      simulations[i].setPosition(physics.position() * direction);
      simulations[i]
          .getRelativeEncoderSim()
          .setPosition(
              physics.position() * direction * (i == 0 ? 1 : config.motorRotationsPerUnit()));
      simulations[i]
          .getRelativeEncoderSim()
          .setVelocity(
              physics.velocity() * direction * (i == 0 ? 1 : 60 * config.motorRotationsPerUnit()));
      if (i == 0 && config.encoderType() == EncoderType.EXTERNAL_SPARK) {
        // REV sensor simulation setters use converted, calibrated feedback units.
        simulations[i].getAbsoluteEncoderSim().setPosition(physics.position());
        simulations[i].getAbsoluteEncoderSim().setVelocity(physics.velocity());
      }
    }
  }

  @Override
  public void updateInputs(PositionJointIOInputs in) {
    sync();
    double battery = RobotController.getBatteryVoltage();
    boolean enabled = DriverStation.isEnabled();
    simulations[0].iterate(physics.velocity(), battery, 0.02);
    double volts = enabled ? simulations[0].getAppliedOutput() * battery : 0;
    // REV simulation applied output is already in the configured positive mechanism direction.
    physics.step(volts, brake, enabled && (!voltageMode || Math.abs(commandedVoltage) > 1e-9));
    sync();
    for (var simulation : simulations) {
      simulation.setBusVoltage(battery);
      simulation.setMotorCurrent(physics.current() / motors.length);
    }
    super.updateInputs(in);
    commonInputs(in, physics.position(), physics.velocity());
    for (int i = 0; i < motors.length; i++) {
      in.motorCurrents[i] = physics.current() / motors.length;
      in.motorVoltages[i] = volts * (i > 0 && config.reversed()[i] ? -1 : 1);
    }
  }

  @Override
  protected double physicalPosition() {
    return physics == null ? super.physicalPosition() : physics.position();
  }

  @Override
  protected void validateGains(PositionJointGains g) {
    physics.validateLimits(g.kMinPosition(), g.kMaxPosition());
  }

  public double physicalPositionForSimulation() {
    return physics.position();
  }

  public double physicalVelocityForSimulation() {
    return physics.velocity();
  }
}
