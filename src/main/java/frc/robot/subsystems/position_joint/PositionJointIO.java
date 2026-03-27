package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface PositionJointIO {
  @AutoLog
  public static class PositionJointIOInputs {
    public double outputPosition = 0.0;
    public double rotorPosition = 0.0;
    public double desiredPosition = 0.0;

    public double velocity = 0.0;
    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};
    public boolean encoderConnected = false;

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(PositionJointIOInputs inputs) {}

  public default void setPosition(double position, double velocity) {}

  public default boolean setPositionDynamic(
      double position, double maxVelocity, double maxAcceleration) {
    return false;
  }

  public default void setVoltage(double voltage) {}

  public default void setGains(PositionJointGains gains) {}

  public default void resetPosition() {}

  public static Supplier<PositionJointIO> replayFactory(String name) {
    return () -> new PositionJointIOReplay(name);
  }

  public static PositionJointIO fromMode(
      String name,
      PositionJointHardwareConfig config,
      Supplier<PositionJointIO> subsystemSupplier,
      DCMotor simMotorModel) {
    return switch (Constants.currentMode) {
      case REAL -> subsystemSupplier.get();
      case SIM -> replayFactory(name).get();
      default -> replayFactory(name).get();
    };
  }

  public static PositionJointIO fromSparkMax(String name, PositionJointHardwareConfig config) {
    return switch (Constants.currentMode) {
      case REAL -> new PositionJointIOSparkMax(name, config);
      case SIM -> new PositionJointIOSimSparkMax(name, config, DCMotor.getNEO(config.canIds().length));
      default -> replayFactory(name).get();
    };
  }

  public static PositionJointIO fromTalonFX(String name, PositionJointHardwareConfig config) {
    return switch (Constants.currentMode) {
      case REAL -> new PositionJointIOTalonFX(name, config);
      case SIM ->
          new PositionJointIOSimTalonFX(
              name, config, DCMotor.getKrakenX60Foc(config.canIds().length));
      default -> replayFactory(name).get();
    };
  }

  public String getName();
}
