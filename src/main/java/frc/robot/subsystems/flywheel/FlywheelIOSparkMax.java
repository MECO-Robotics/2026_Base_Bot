package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;

/** SparkMax-backed implementation of {@link FlywheelIO}. */
public class FlywheelIOSparkMax implements FlywheelIO {
  private final String name;

  private final SparkMax[] motors;
  private final SparkBaseConfig leaderConfig;

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private final Alert[] motorAlerts;

  private double velocitySetpoint = 0.0;

  /**
   * Creates a SparkMax flywheel IO implementation.
   *
   * @param name subsystem/logging name
   * @param config hardware mapping and mechanism constants
   */
  public FlywheelIOSparkMax(String name, FlywheelHardwareConfig config) {
    this(name, config, true);
  }

  /**
   * Creates a SparkMax flywheel IO implementation with selectable motor type.
   *
   * @param name subsystem/logging name
   * @param config hardware mapping and mechanism constants
   * @param isBrushless true for NEO/brushless mode, false for brushed mode
   */
  public FlywheelIOSparkMax(String name, FlywheelHardwareConfig config, boolean isBrushless) {
    this.name = name;

    int numMotors = config.canIds().length;

    assert numMotors > 0 && (numMotors == config.reversed().length);

    motors = new SparkMax[numMotors];
    motorsConnected = new boolean[numMotors];
    motorPositions = new double[numMotors];
    motorVelocities = new double[numMotors];
    motorVoltages = new double[numMotors];
    motorCurrents = new double[numMotors];
    motorAlerts = new Alert[numMotors];

    motors[0] =
        new SparkMax(config.canIds()[0], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
    if (isBrushless) {
      leaderConfig =
          new SparkMaxConfig()
              .apply(
                  new EncoderConfig()
                      .positionConversionFactor(1.0 / config.gearRatio())
                      .velocityConversionFactor(1.0 / (60.0 * config.gearRatio())))
              .inverted(config.reversed()[0])
              .smartCurrentLimit(config.currentLimit());

    } else {
      // NOTE: Brushed Motors does not support current limits! BE CAREFUL
      leaderConfig =
          new SparkMaxConfig()
              .apply(
                  new EncoderConfig()
                      .positionConversionFactor(1.0 / config.gearRatio())
                      .velocityConversionFactor(1.0 / (60.0 * config.gearRatio()))
                      .inverted(config.reversed()[0]));
    }

    motors[0].configure(
        leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    motorAlerts[0] =
        new Alert(
            name,
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0],
            AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] =
          new SparkMax(config.canIds()[i], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
      motors[i].configure(
          new SparkMaxConfig()
              .follow(motors[0], config.reversed()[i])
              .smartCurrentLimit(config.currentLimit()),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);

      motorAlerts[i] =
          new Alert(
              name,
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocity = motors[0].getEncoder().getVelocity();
    inputs.position = motors[0].getEncoder().getPosition();

    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] = motors[i].getLastError() == REVLibError.kOk;

      motorPositions[i] = motors[i].getEncoder().getPosition();
      motorVelocities[i] = motors[i].getEncoder().getVelocity();

      motorVoltages[i] = motors[i].getAppliedOutput() * motors[i].getBusVoltage();
      motorCurrents[i] = motors[i].getOutputCurrent();

      motorAlerts[i].set(!motorsConnected[i]);
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorSupplyVoltages =
        java.util.Arrays.stream(motors).mapToDouble(m -> m.getBusVoltage()).toArray();
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;

    motors[0]
        .getClosedLoopController()
        .setSetpoint(velocitySetpoint, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setVoltage(voltage);
  }

  @Override
  public void setGains(FlywheelGains gains) {
    motors[0].configure(
        leaderConfig
            .voltageCompensation(12)
            .apply(
                new ClosedLoopConfig()
                    .pid(gains.kP() / 12, gains.kI() / 12000, gains.kD() * 1000 / 12)
                    .apply(
                        new com.revrobotics.spark.config.FeedForwardConfig()
                            .kS(gains.kS())
                            .kV(gains.kV())
                            .kA(gains.kA()))
                    .apply(new MAXMotionConfig().maxAcceleration(gains.kMaxAccel()))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    System.out.println(name + " gains set to " + gains);
  }

  /** Returns this flywheel's loggable subsystem name. */
  @Override
  public String getName() {
    return name;
  }
}
