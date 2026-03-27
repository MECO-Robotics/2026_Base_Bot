package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.commands.position_joint.PositionJointVelocityCommand;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PositionJoint extends SubsystemBase {
  private final PositionJointIO positionJoint;
  private final PositionJointIOInputsAutoLogged inputs = new PositionJointIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kG;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber kMaxVelo;
  private final LoggedTunableNumber kMaxAccel;

  private final LoggedTunableNumber kMinPosition;
  private final LoggedTunableNumber kMaxPosition;

  private final LoggedTunableNumber kTolerance;

  private final LoggedTunableNumber kSetpoint;
  private Double profileMaxVelocityOverride = null;
  private double goalPosition;

  public PositionJoint(PositionJointIO io, PositionJointGains gains) {
    super(io.getName());

    positionJoint = io;
    name = positionJoint.getName();

    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kG = new LoggedTunableNumber(name + "/Gains/kG", gains.kG());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());

    kMaxVelo = new LoggedTunableNumber(name + "/Gains/kMaxVelo", gains.kMaxVelo());
    kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());

    kMinPosition = new LoggedTunableNumber(name + "/Gains/kMinPosition", gains.kMinPosition());
    kMaxPosition = new LoggedTunableNumber(name + "/Gains/kMaxPosition", gains.kMaxPosition());

    kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());

    kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", gains.kDefaultSetpoint());
    goalPosition = gains.kDefaultSetpoint();

    positionJoint.setGains(gains);

    SmartDashboard.putData(name, this);
  }

  @Override
  public void periodic() {
    positionJoint.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    boolean usingDynamicOverride =
        profileMaxVelocityOverride != null
            && positionJoint.setPositionDynamic(
                goalPosition, profileMaxVelocityOverride, kMaxAccel.get());
    if (!usingDynamicOverride) {
      positionJoint.setPosition(goalPosition, 0.0);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          positionJoint.setGains(
              new PositionJointGains(
                  values[0],
                  values[1],
                  values[2],
                  values[3],
                  values[4],
                  values[5],
                  values[6],
                  values[7],
                  values[8],
                  values[9],
                  values[10],
                  values[11],
                  values[12]));

          goalPosition = MathUtil.clamp(values[12], kMinPosition.get(), kMaxPosition.get());
        },
        kP,
        kI,
        kD,
        kS,
        kG,
        kV,
        kA,
        kMaxVelo,
        kMaxAccel,
        kMinPosition,
        kMaxPosition,
        kTolerance,
        kSetpoint);

    Logger.recordOutput(name + "/isFinished", isFinished());
  }

  public void setPosition(double position) {
    goalPosition = MathUtil.clamp(position, kMinPosition.get(), kMaxPosition.get());
  }

  public void setPosition(double position, double maxVelocity) {
    profileMaxVelocityOverride = Math.max(0.0, maxVelocity);
    setPosition(position);
  }

  public void clearProfileConstraintsOverride() {
    if (profileMaxVelocityOverride == null) {
      return;
    }

    profileMaxVelocityOverride = null;
  }

  public void incrementPosition(double deltaPosition) {
    setPosition(goalPosition + deltaPosition);
  }

  public void setVoltage(double voltage) {
    positionJoint.setVoltage(voltage);
  }

  public double getPosition() {
    return inputs.outputPosition;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getDesiredPosition() {
    return inputs.desiredPosition;
  }

  public boolean isFinished() {
    return Math.abs(inputs.outputPosition - goalPosition) < kTolerance.get();
  }

  public void resetPosition() {
    positionJoint.resetPosition();
    goalPosition = 0;
  }

  public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier) {
    return new PositionJointPositionCommand(positionJoint, positionSupplier);
  }

  public static Command setPosition(
      PositionJoint positionJoint,
      DoubleSupplier positionSupplier,
      DoubleSupplier maxVelocitySupplier) {
    return new PositionJointPositionCommand(positionJoint, positionSupplier, maxVelocitySupplier);
  }

  public static Command setVelocity(PositionJoint positionJoint, DoubleSupplier velocitySupplier) {
    return new PositionJointVelocityCommand(positionJoint, velocitySupplier);
  }
}
