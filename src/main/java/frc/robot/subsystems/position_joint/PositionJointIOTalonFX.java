package frc.robot.subsystems.position_joint;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.constants.types.PositionJointConstants.*;
import java.util.function.DoubleSupplier;

/** Phoenix joint IO, also used unchanged by the vendor simulation implementation. */
public class PositionJointIOTalonFX extends JointControl {
  protected final TalonFX[] motors;
  protected final CANcoder encoder;
  private final Alert[] alerts;
  private final Alert encoderAlert;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private double appliedVelocity = Double.NaN, appliedAcceleration = Double.NaN;

  public PositionJointIOTalonFX(String name, PositionJointHardwareConfig config) {
    this(name, config, () -> 0);
  }

  public PositionJointIOTalonFX(
      String name, PositionJointHardwareConfig config, DoubleSupplier ff) {
    super(name, config, ff);
    if (config.encoderType() == EncoderType.EXTERNAL_DIO
        || config.encoderType() == EncoderType.EXTERNAL_SPARK)
      throw new IllegalArgumentException("Talon joints support INTERNAL or CANcoder feedback");
    CANBus bus = new CANBus(config.canBus());
    if (config.encoderType() == EncoderType.INTERNAL) encoder = null;
    else {
      encoder = new CANcoder(config.encoderID(), bus);
      var e = config.encoder();
      tryUntilOk(
          5,
          () ->
              encoder
                  .getConfigurator()
                  .apply(
                      new CANcoderConfiguration()
                          .withMagnetSensor(
                              new MagnetSensorConfigs()
                                  .withSensorDirection(
                                      e.reversed()
                                          ? SensorDirectionValue.Clockwise_Positive
                                          : SensorDirectionValue.CounterClockwise_Positive)
                                  .withMagnetOffset(e.offsetRotations()))));
    }
    encoderAlert = new Alert(name, "Joint encoder disconnected", Alert.AlertType.kError);
    motors = new TalonFX[config.canIds().length];
    alerts = new Alert[motors.length];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonFX(config.canIds()[i], bus);
      final int index = i;
      var c =
          new TalonFXConfiguration()
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      .withSupplyCurrentLimit(config.currentLimit())
                      .withSupplyCurrentLimitEnable(true))
              .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
      if (i == 0) {
        c.MotorOutput.Inverted =
            config.reversed()[0]
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        c.Feedback.SensorToMechanismRatio = config.motorRotationsPerUnit();
        if (encoder != null) {
          c.Feedback.FeedbackRemoteSensorID = config.encoderID();
          c.Feedback.SensorToMechanismRatio = 1 / config.encoder().unitsPerRotation();
          c.Feedback.RotorToSensorRatio =
              config.motorRotationsPerUnit() * config.encoder().unitsPerRotation();
          c.Feedback.FeedbackSensorSource =
              config.encoderType() == EncoderType.EXTERNAL_CANCODER_PRO
                  ? FeedbackSensorSourceValue.FusedCANcoder
                  : FeedbackSensorSourceValue.RemoteCANcoder;
        }
      }
      tryUntilOk(5, () -> motors[index].getConfigurator().apply(c));
      if (i > 0)
        motors[i].setControl(
            new Follower(
                motors[0].getDeviceID(),
                config.reversed()[i] ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
      alerts[i] =
          new Alert(
              name, "Joint motor disconnected: " + config.canIds()[i], Alert.AlertType.kError);
    }
  }

  @Override
  protected double physicalPosition() {
    return motors[0].getPosition().refresh().getValueAsDouble();
  }

  @Override
  public void updateInputs(PositionJointIOInputs in) {
    var position = motors[0].getPosition();
    var velocity = motors[0].getVelocity();
    BaseStatusSignal.refreshAll(position, velocity);
    commonInputs(in, position.getValueAsDouble(), velocity.getValueAsDouble());
    in.motorsConnected = new boolean[motors.length];
    in.motorPositions = new double[motors.length];
    in.motorVelocities = new double[motors.length];
    in.motorVoltages = new double[motors.length];
    in.motorCurrents = new double[motors.length];
    in.motorSupplyVoltages = new double[motors.length];
    for (int i = 0; i < motors.length; i++) {
      var p = motors[i].getRotorPosition();
      var v = motors[i].getRotorVelocity();
      var volts = motors[i].getMotorVoltage();
      var supply = motors[i].getSupplyVoltage();
      var current = motors[i].getStatorCurrent();
      in.motorsConnected[i] = BaseStatusSignal.refreshAll(p, v, volts, supply, current).isOK();
      alerts[i].set(!in.motorsConnected[i]);
      in.motorPositions[i] = p.getValueAsDouble();
      in.motorVelocities[i] = v.getValueAsDouble();
      in.motorVoltages[i] = volts.getValueAsDouble();
      in.motorSupplyVoltages[i] = supply.getValueAsDouble();
      in.motorCurrents[i] = current.getValueAsDouble();
    }
    in.rotorPosition = in.motorPositions[0];
    in.encoderConnected =
        encoder != null && encoder.getAbsolutePosition().refresh().getStatus().isOK();
    encoderAlert.set(encoder != null && !in.encoderConnected);
  }

  public static Slot0Configs gainConfiguration(
      PositionJointHardwareConfig c, PositionJointGains g) {
    return new Slot0Configs()
        .withKP(g.kP())
        .withKI(g.kI())
        .withKD(g.kD())
        .withKS(g.kS())
        .withKV(g.kV())
        .withKA(g.kA())
        .withKG(c.gravityType() == GravityType.SINE ? 0 : g.kG())
        .withGravityType(
            c.gravityType() == GravityType.CONSTANT
                ? GravityTypeValue.Elevator_Static
                : GravityTypeValue.Arm_Cosine);
  }

  @Override
  protected void configureGains() {
    tryUntilOk(5, () -> motors[0].getConfigurator().apply(gainConfiguration(config, gains)));
    tryUntilOk(
        5,
        () ->
            motors[0]
                .getConfigurator()
                .apply(
                    new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(gains.kMaxPosition())
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(gains.kMinPosition())));
  }

  @Override
  protected void requestPosition(double p, double v, double a, double ff) {
    if (appliedVelocity != v || appliedAcceleration != a) {
      tryUntilOk(
          5,
          () ->
              motors[0]
                  .getConfigurator()
                  .apply(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(v)
                          .withMotionMagicAcceleration(a)));
      appliedVelocity = v;
      appliedAcceleration = a;
    }
    motors[0].setControl(request.withPosition(p).withFeedForward(ff));
  }

  @Override
  protected void requestVoltage(double volts) {
    motors[0].setControl(new VoltageOut(volts));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brake == enabled) return;
    brake = enabled;
    for (var m : motors)
      m.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void close() {
    for (var m : motors) m.close();
    if (encoder != null) encoder.close();
  }
}
