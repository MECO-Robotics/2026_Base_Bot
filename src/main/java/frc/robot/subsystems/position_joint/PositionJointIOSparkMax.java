package frc.robot.subsystems.position_joint;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.constants.types.PositionJointConstants.*;
import java.util.function.DoubleSupplier;

/** Spark closed-loop IO. Physical feedback remains calibrated when the user reference is reset. */
public class PositionJointIOSparkMax extends JointControl {
  protected final SparkMax[] motors;
  private double appliedVelocity = Double.NaN, appliedAcceleration = Double.NaN;
  private final Alert[] alerts;

  public PositionJointIOSparkMax(String name, PositionJointHardwareConfig config) {
    this(name, config, () -> 0, true);
  }

  public PositionJointIOSparkMax(
      String name, PositionJointHardwareConfig config, DoubleSupplier ff, boolean brushless) {
    super(name, config, ff);
    if (!brushless)
      throw new IllegalArgumentException(
          "Position joints require supported brushless feedback/current limiting");
    if (config.encoderType() != EncoderType.INTERNAL
        && config.encoderType() != EncoderType.EXTERNAL_SPARK)
      throw new IllegalArgumentException(
          "Spark joints support INTERNAL or EXTERNAL_SPARK feedback; use Talon for CANcoder");
    motors = new SparkMax[config.canIds().length];
    alerts = new Alert[motors.length];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new SparkMax(config.canIds()[i], SparkLowLevel.MotorType.kBrushless);
      SparkMaxConfig c = new SparkMaxConfig();
      c.smartCurrentLimit(config.currentLimit()).idleMode(SparkBaseConfig.IdleMode.kBrake);
      if (i == 0) {
        c.inverted(config.reversed()[0]);
        c.encoder
            .positionConversionFactor(1 / config.motorRotationsPerUnit())
            .velocityConversionFactor(1 / (60 * config.motorRotationsPerUnit()));
        if (config.encoderType() == EncoderType.EXTERNAL_SPARK) {
          var e = config.encoder();
          c.absoluteEncoder
              .positionConversionFactor(e.unitsPerRotation())
              .velocityConversionFactor(e.unitsPerRotation() / 60)
              .zeroOffset(((e.reversed() ? e.offsetRotations() : -e.offsetRotations()) % 1 + 1) % 1)
              .inverted(e.reversed());
          c.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        } else c.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      } else c.follow(motors[0], config.reversed()[i]);
      motors[i].configure(c, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      alerts[i] =
          new Alert(
              name, "Joint motor disconnected: " + config.canIds()[i], Alert.AlertType.kError);
    }
  }

  @Override
  protected double physicalPosition() {
    return config.encoderType() == EncoderType.EXTERNAL_SPARK
        ? motors[0].getAbsoluteEncoder().getPosition()
        : motors[0].getEncoder().getPosition();
  }

  protected double physicalVelocity() {
    return config.encoderType() == EncoderType.EXTERNAL_SPARK
        ? motors[0].getAbsoluteEncoder().getVelocity()
        : motors[0].getEncoder().getVelocity();
  }

  @Override
  public void updateInputs(PositionJointIOInputs in) {
    commonInputs(in, physicalPosition(), physicalVelocity());
    in.motorsConnected = new boolean[motors.length];
    in.motorPositions = new double[motors.length];
    in.motorVelocities = new double[motors.length];
    in.motorVoltages = new double[motors.length];
    in.motorCurrents = new double[motors.length];
    in.motorSupplyVoltages = new double[motors.length];
    for (int i = 0; i < motors.length; i++) {
      in.motorPositions[i] =
          motors[i].getEncoder().getPosition() * (i == 0 ? config.motorRotationsPerUnit() : 1);
      in.motorVelocities[i] =
          motors[i].getEncoder().getVelocity()
              * (i == 0 ? config.motorRotationsPerUnit() : 1.0 / 60);
      in.motorSupplyVoltages[i] = motors[i].getBusVoltage();
      in.motorVoltages[i] = motors[i].getAppliedOutput() * in.motorSupplyVoltages[i];
      in.motorCurrents[i] = motors[i].getOutputCurrent();
      in.motorsConnected[i] = motors[i].getLastError() == REVLibError.kOk;
      alerts[i].set(!in.motorsConnected[i]);
    }
    in.rotorPosition = in.motorPositions[0];
    in.encoderConnected = config.encoderType() != EncoderType.INTERNAL && in.motorsConnected[0];
  }
  /** Testable voltage-based gains translated to Spark's duty-cycle PID at nominal 12V. */
  public static SparkMaxConfig gainConfiguration(
      PositionJointHardwareConfig hardware, PositionJointGains g) {
    SparkMaxConfig c = new SparkMaxConfig();
    c.voltageCompensation(12);
    c.closedLoop.pid(g.kP() / 12, g.kI() / 12000, g.kD() * 1000 / 12);
    c.closedLoop.feedForward.kS(g.kS()).kV(g.kV()).kA(g.kA());
    if (hardware.gravityType() == GravityType.CONSTANT) c.closedLoop.feedForward.kG(g.kG());
    else
      c.closedLoop
          .feedForward
          .kCos(hardware.gravityType() == GravityType.COSINE ? g.kG() : 0)
          .kCosRatio(1);
    c.closedLoop
        .maxMotion
        .cruiseVelocity(g.kMaxVelo())
        .maxAcceleration(g.kMaxAccel())
        .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);
    c.softLimit
        .forwardSoftLimit(g.kMaxPosition())
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(g.kMinPosition())
        .reverseSoftLimitEnabled(true);
    return c;
  }

  @Override
  protected void configureGains() {
    motors[0].configure(
        gainConfiguration(config, gains),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    appliedVelocity = gains.kMaxVelo();
    appliedAcceleration = gains.kMaxAccel();
  }

  @Override
  protected void requestPosition(double position, double velocity, double acceleration, double ff) {
    if (velocity != appliedVelocity || acceleration != appliedAcceleration) {
      var c = new SparkMaxConfig();
      c.closedLoop.maxMotion.cruiseVelocity(velocity).maxAcceleration(acceleration);
      // Complete the constraint update before issuing the request that depends on it.
      motors[0].configure(c, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      appliedVelocity = velocity;
      appliedAcceleration = acceleration;
    }
    motors[0]
        .getClosedLoopController()
        .setSetpoint(
            position, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  protected void requestVoltage(double volts) {
    motors[0].setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brake == enabled) return;
    brake = enabled;
    for (var motor : motors)
      motor.configure(
          new SparkMaxConfig()
              .idleMode(
                  enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
  }

  @Override
  public void close() {
    for (var motor : motors) motor.close();
  }
}
