package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;
import frc.robot.constants.types.PositionJointConstants.JointSimulationConfig;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a closed-loop position-controlled joint. */
public interface PositionJointIO {
  /**
   * Logged inputs shared by all joint implementations.
   *
   * <p>Output units are rotations and rotations/sec for angular joints, metres and metres/sec for
   * linear joints. Motor telemetry uses rotor rotations and rotations/sec.
   */
  @AutoLog
  public static class PositionJointIOInputs {
    /** Joint output position after gearing (mechanism position). */
    public double outputPosition = 0.0;
    /** Motor rotor position before gearing (motor shaft position). */
    public double rotorPosition = 0.0;
    /** Last requested joint position setpoint. */
    public double desiredPosition = 0.0;

    /** Measured mechanism velocity. */
    public double velocity = 0.0;
    /** Last requested mechanism velocity setpoint. */
    public double desiredVelocity = 0.0;

    /** Connectivity state per motor controller. */
    public boolean[] motorsConnected = {false};
    /** True when an external encoder is available and healthy. */
    public boolean encoderConnected = false;

    /** Per-motor position telemetry. */
    public double[] motorPositions = {0.0};
    /** Per-motor velocity telemetry. */
    public double[] motorVelocities = {0.0};
    /** Per-motor acceleration telemetry (optional, impl-dependent). */
    public double[] motorAccelerations = {0.0};

    /** Per-motor applied voltage telemetry. */
    public double[] motorVoltages = {0.0};

    /** Per-controller measured supply voltage, separately from applied motor voltage. */
    public double[] motorSupplyVoltages = {0.0};
    /** Per-motor current draw telemetry. */
    public double[] motorCurrents = {0.0};
  }

  /** Refreshes all sensor and diagnostic inputs. */
  public default void updateInputs(PositionJointIOInputs inputs) {}

  /** Commands a joint position with the configured default profile constraints. */
  public default void setPosition(double position) {}

  /**
   * Commands a dynamic closed-loop position request with runtime profile constraints.
   *
   * @return true if the IO implementation handled the request directly
   */
  public default boolean setPositionDynamic(
      double position, double maxVelocity, double maxAcceleration) {
    return false;
  }

  /** Commands an open-loop voltage output. */
  public default void setVoltage(double voltage) {}

  /** Sets whether motor neutral mode should brake or coast when output is zero. */
  public default void setBrakeMode(boolean enabled) {}

  /** Applies controller/feedforward gains. */
  public default void setGains(PositionJointGains gains) {}

  /** Zeros the reported reference without changing physical position, velocity, or calibration. */
  public default void resetPosition() {}

  public default double getPositionOffset() {
    return 0;
  }

  /** Creates a replay position-joint IO supplier. */
  public static Supplier<PositionJointIO> replayFactory(String name) {
    return () ->
        new PositionJointIO() {
          @Override
          public String getName() {
            return name;
          }
        };
  }

  /** Selects an implementation without constructing unused hardware or physics. */
  public static PositionJointIO fromMode(
      String name, Supplier<PositionJointIO> real, Supplier<PositionJointIO> sim) {
    return selectMode(Constants.currentMode, name, real, sim);
  }

  public static PositionJointIO selectMode(
      Constants.Mode mode,
      String name,
      Supplier<PositionJointIO> real,
      Supplier<PositionJointIO> sim) {
    return switch (mode) {
      case REAL -> real.get();
      case SIM -> sim.get();
      case REPLAY -> replayFactory(name).get();
    };
  }

  /** Creates mode-appropriate position-joint IO using SparkMax for real hardware. */
  public static PositionJointIO fromSparkMax(
      String name, PositionJointHardwareConfig config, JointSimulationConfig simulation) {
    return switch (Constants.currentMode) {
      case REAL -> new PositionJointIOSparkMax(name, config);
      case SIM -> new PositionJointIOSimSparkMax(
          name, config, simulation, DCMotor.getNEO(config.canIds().length));
      default -> replayFactory(name).get();
    };
  }

  /** Creates mode-appropriate position-joint IO using TalonFX for real hardware. */
  public static PositionJointIO fromTalonFX(
      String name, PositionJointHardwareConfig config, JointSimulationConfig simulation) {
    return switch (Constants.currentMode) {
      case REAL -> new PositionJointIOTalonFX(name, config);
      case SIM -> new PositionJointIOSimTalonFX(
          name, config, simulation, DCMotor.getKrakenX60Foc(config.canIds().length));
      default -> replayFactory(name).get();
    };
  }

  /** Returns a unique telemetry/logging name for this joint. */
  public String getName();
}
