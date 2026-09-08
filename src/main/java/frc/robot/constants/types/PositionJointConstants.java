package frc.robot.constants.types;

import java.util.Objects;

/** Angular positions are rotations; linear positions are metres. All rates are per second. */
public final class PositionJointConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    SINE
  }

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_CANCODER_PRO,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  public enum MechanismType {
    ROTATIONAL,
    LINEAR
  }

  /** Gains use volts per mechanism unit (and its time derivatives), for both vendors. */
  public record PositionJointGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance,
      double kDefaultSetpoint) {
    public PositionJointGains {
      for (double v :
          new double[] {
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
            kDefaultSetpoint
          }) if (!Double.isFinite(v)) throw new IllegalArgumentException("Gains must be finite");
      if (kMaxVelo <= 0
          || kMaxAccel <= 0
          || kTolerance <= 0
          || kMinPosition >= kMaxPosition
          || kDefaultSetpoint < kMinPosition
          || kDefaultSetpoint > kMaxPosition)
        throw new IllegalArgumentException("Invalid joint constraints");
    }
  }

  /** Calibrated units = (directed sensor rotations + offsetRotations) * unitsPerRotation. */
  public record EncoderCalibration(
      EncoderType type, int id, double unitsPerRotation, double offsetRotations, boolean reversed) {
    public EncoderCalibration {
      Objects.requireNonNull(type);
      if ((type == EncoderType.INTERNAL
              && (unitsPerRotation != 1 || offsetRotations != 0 || reversed))
          || ((type == EncoderType.EXTERNAL_CANCODER || type == EncoderType.EXTERNAL_CANCODER_PRO)
              && id > 62)
          || id < 0
          || !Double.isFinite(unitsPerRotation)
          || unitsPerRotation <= 0
          || !Double.isFinite(offsetRotations))
        throw new IllegalArgumentException("Invalid encoder calibration");
    }

    public static EncoderCalibration internal() {
      return new EncoderCalibration(EncoderType.INTERNAL, 0, 1, 0, false);
    }

    public double calibrate(double rawRotations) {
      return ((reversed ? -rawRotations : rawRotations) + offsetRotations) * unitsPerRotation;
    }
  }

  /** gearRatio is motor rotations/output-shaft rotation. Calibration and physics are separate. */
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      int currentLimit,
      String canBus,
      MechanismType mechanismType,
      GravityType gravityType,
      double outputRadiusMeters,
      EncoderCalibration encoder) {
    public PositionJointHardwareConfig {
      MotorConfigValidation.validate(canIds, reversed, gearRatio, currentLimit, canBus);
      canIds = canIds.clone();
      reversed = reversed.clone();
      Objects.requireNonNull(mechanismType);
      Objects.requireNonNull(gravityType);
      Objects.requireNonNull(encoder);
      if (!Double.isFinite(outputRadiusMeters)
          || outputRadiusMeters < 0
          || (mechanismType == MechanismType.LINEAR
              && (outputRadiusMeters <= 0 || gravityType != GravityType.CONSTANT)))
        throw new IllegalArgumentException(
            "Linear joints require a drum radius and CONSTANT gravity");
    }

    @Override
    public int[] canIds() {
      return canIds.clone();
    }

    @Override
    public boolean[] reversed() {
      return reversed.clone();
    }

    public double unitsPerOutputRotation() {
      return mechanismType == MechanismType.LINEAR ? 2 * Math.PI * outputRadiusMeters : 1;
    }

    public double motorRotationsPerUnit() {
      return gearRatio / unitsPerOutputRotation();
    }

    public EncoderType encoderType() {
      return encoder.type();
    }

    public int encoderID() {
      return encoder.id();
    }

    public double gravityVoltage(double physicalPosition, double kG) {
      return switch (gravityType) {
        case CONSTANT -> kG;
        case COSINE -> kG * Math.cos(physicalPosition * 2 * Math.PI);
        case SINE -> kG * Math.sin(physicalPosition * 2 * Math.PI);
      };
    }
  }

  /** Physical limits are in mechanism units, inertia at the output shaft, mass in kg. */
  public record JointSimulationConfig(
      double outputInertiaKgMetersSquared,
      double massKg,
      double armLengthMeters,
      double minPosition,
      double maxPosition,
      double initialPosition) {
    public JointSimulationConfig {
      for (double v :
          new double[] {
            outputInertiaKgMetersSquared,
            massKg,
            armLengthMeters,
            minPosition,
            maxPosition,
            initialPosition
          })
        if (!Double.isFinite(v))
          throw new IllegalArgumentException("Simulation parameters must be finite");
      if (outputInertiaKgMetersSquared <= 0
          || massKg <= 0
          || armLengthMeters <= 0
          || minPosition >= maxPosition
          || initialPosition < minPosition
          || initialPosition > maxPosition)
        throw new IllegalArgumentException("Invalid physical simulation parameters");
    }

    public void validateLimits(double min, double max) {
      if (min < minPosition || max > maxPosition)
        throw new IllegalArgumentException("Software limits exceed physical travel");
    }
  }
  // Illustrative values only: measure and tune the actual mechanism before enabling it.
  public static final PositionJointGains EXAMPLE_GAINS =
      new PositionJointGains(4, 0, 0, 0, 0, 0, 0, 1, 2, 0, 0.5, 0.01, 0);
  public static final PositionJointHardwareConfig EXAMPLE_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10},
          new boolean[] {false},
          85.33333,
          40,
          "",
          MechanismType.ROTATIONAL,
          GravityType.COSINE,
          0,
          EncoderCalibration.internal());
  public static final JointSimulationConfig EXAMPLE_SIMULATION =
      new JointSimulationConfig(0.025, 1, 0.5, 0, 0.5, 0);
}
