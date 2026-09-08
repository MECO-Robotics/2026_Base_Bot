package frc.robot.subsystems.position_joint;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import frc.robot.constants.types.PositionJointConstants.*;
import frc.robot.sim.JointPhysics;

/** Real Phoenix control and calibration backed by a physical plant. */
public class PositionJointIOSimTalonFX extends PositionJointIOTalonFX {
  private final JointPhysics physics;

  public PositionJointIOSimTalonFX(
      String name,
      PositionJointHardwareConfig config,
      JointSimulationConfig simulation,
      DCMotor motor) {
    super(name, config);
    physics = new JointPhysics(config, simulation, motor);
    for (int i = 0; i < motors.length; i++)
      motors[i].getSimState().Orientation =
          (config.reversed()[0] ^ (i > 0 && config.reversed()[i]))
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    sync();
  }

  private void sync() {
    for (var m : motors) {
      var sim = m.getSimState();
      sim.setSupplyVoltage(RobotController.getBatteryVoltage());
      sim.setRawRotorPosition(physics.position() * config.motorRotationsPerUnit());
      sim.setRotorVelocity(physics.velocity() * config.motorRotationsPerUnit());
    }
    if (encoder != null) {
      var c = config.encoder();
      double sign = c.reversed() ? -1 : 1;
      encoder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
      encoder
          .getSimState()
          .setRawPosition((physics.position() / c.unitsPerRotation() - c.offsetRotations()) * sign);
      encoder.getSimState().setVelocity(physics.velocity() / c.unitsPerRotation() * sign);
    }
  }

  @Override
  public void updateInputs(PositionJointIOInputs in) {
    sync();
    boolean enabled = DriverStation.isEnabled();
    double volts = enabled ? motors[0].getSimState().getMotorVoltage() : 0;
    physics.step(volts, brake, enabled && (!voltageMode || Math.abs(commandedVoltage) > 1e-9));
    sync();
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
