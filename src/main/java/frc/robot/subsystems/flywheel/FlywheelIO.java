package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.sim.flywheel.FlywheelIOSimSparkMax;
import frc.robot.sim.flywheel.FlywheelIOSimTalonFX;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocity = 0.0;
    public double desiredVelocity = 0.0;

    public double position = 0.0;

    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(FlywheelGains gains) {}

  public static Supplier<FlywheelIO> replayFactory(String name) {
    return () -> new FlywheelIOReplay(name);
  }

  public static FlywheelIO fromMode(
      String name,
      FlywheelHardwareConfig config,
      Supplier<FlywheelIO> subsystemSupplier,
      DCMotor simMotorModel) {
    return switch (Constants.currentMode) {
      case REAL -> subsystemSupplier.get();
      case SIM -> replayFactory(name).get();
      default -> replayFactory(name).get();
    };
  }

  public static FlywheelIO fromSparkMax(String name, FlywheelHardwareConfig config) {
    return switch (Constants.currentMode) {
      case REAL -> new FlywheelIOSparkMax(name, config);
      case SIM -> new FlywheelIOSimSparkMax(name, config, DCMotor.getNEO(config.canIds().length));
      default -> replayFactory(name).get();
    };
  }

  public static FlywheelIO fromTalonFX(String name, FlywheelHardwareConfig config) {
    return switch (Constants.currentMode) {
      case REAL -> new FlywheelIOTalonFX(name, config);
      case SIM -> new FlywheelIOSimTalonFX(name, config, DCMotor.getKrakenX60Foc(config.canIds().length));
      default -> replayFactory(name).get();
    };
  }

  public String getName();
}
