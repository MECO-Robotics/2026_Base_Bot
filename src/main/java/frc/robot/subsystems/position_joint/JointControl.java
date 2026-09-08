package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.types.PositionJointConstants.*;
import java.util.function.DoubleSupplier;

/** One control/reference implementation shared by hardware and vendor simulation subclasses. */
public abstract class JointControl implements PositionJointIO, AutoCloseable {
  protected final String name;
  protected final PositionJointHardwareConfig config;
  protected final DoubleSupplier externalFeedforward;
  protected PositionJointGains gains;
  protected double zeroOffset;
  protected double target;
  protected boolean brake = true;
  protected boolean voltageMode = false;
  protected double commandedVoltage;

  protected JointControl(
      String name, PositionJointHardwareConfig config, DoubleSupplier feedforward) {
    this.name = name;
    this.config = config;
    this.externalFeedforward = java.util.Objects.requireNonNull(feedforward);
  }

  protected abstract double physicalPosition();

  protected abstract void configureGains();

  protected abstract void requestPosition(
      double physicalPosition, double velocity, double acceleration, double arbitraryVolts);

  protected abstract void requestVoltage(double volts);

  protected void validateGains(PositionJointGains value) {}

  @Override
  public final void setGains(PositionJointGains value) {
    validateGains(value);
    gains = value;
    configureGains();
  }

  @Override
  public final void setPosition(double position) {
    requireGains();
    commandPosition(position, gains.kMaxVelo(), gains.kMaxAccel());
  }

  @Override
  public final boolean setPositionDynamic(double position, double velocity, double acceleration) {
    requireGains();
    if (!Double.isFinite(velocity)
        || !Double.isFinite(acceleration)
        || velocity <= 0
        || acceleration <= 0)
      throw new IllegalArgumentException("Profile constraints must be positive");
    commandPosition(position, velocity, acceleration);
    return true;
  }

  private void commandPosition(double position, double velocity, double acceleration) {
    if (!Double.isFinite(position)) throw new IllegalArgumentException("Position must be finite");
    voltageMode = false;
    double physical =
        MathUtil.clamp(position + zeroOffset, gains.kMinPosition(), gains.kMaxPosition());
    target = physical - zeroOffset;
    double feedforward = externalFeedforward.getAsDouble() + softwareGravity();
    if (!Double.isFinite(feedforward))
      throw new IllegalArgumentException("Feedforward must be finite");
    requestPosition(physical, velocity, acceleration, feedforward);
  }
  // Only SINE needs software gravity. The vendors handle CONSTANT and COSINE in their slots.
  protected double softwareGravity() {
    return config.gravityType() == GravityType.SINE
        ? config.gravityVoltage(physicalPosition(), gains.kG())
        : 0;
  }

  @Override
  public final void setVoltage(double volts) {
    if (!Double.isFinite(volts)) throw new IllegalArgumentException("Voltage must be finite");
    voltageMode = true;
    commandedVoltage = volts;
    requestVoltage(volts);
  }

  @Override
  public final void resetPosition() {
    zeroOffset = physicalPosition();
    target = 0;
    voltageMode = false;
    commandedVoltage = 0;
    setBrakeMode(true);
    requestVoltage(0);
    if (gains != null) setPosition(0);
  }

  @Override
  public final double getPositionOffset() {
    return zeroOffset;
  }

  @Override
  public final String getName() {
    return name;
  }

  protected void requireGains() {
    if (gains == null) throw new IllegalStateException("Set gains before controlling joint");
  }

  protected void commonInputs(PositionJointIOInputs in, double physical, double velocity) {
    in.outputPosition = physical - zeroOffset;
    in.velocity = velocity;
    in.desiredPosition = target;
    in.desiredVelocity = 0;
    in.rotorPosition = physical * config.motorRotationsPerUnit();
  }
}
