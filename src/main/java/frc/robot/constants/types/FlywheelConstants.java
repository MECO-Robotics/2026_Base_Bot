package frc.robot.constants.types;

/** Shared constants and configuration records for flywheel/roller mechanisms. */
public class FlywheelConstants {
  /** Closed-loop and feedforward gains for one flywheel mechanism. */
  public record FlywheelGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {
    public FlywheelGains {
      for (double value : new double[] {kP, kI, kD, kS, kV, kA, kMaxAccel, kTolerance})
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Gains must be finite");
      if (kMaxAccel <= 0 || kTolerance <= 0)
        throw new IllegalArgumentException("Invalid flywheel constraints");
    }
  }

  /**
   * Hardware mapping and mechanical constants for one flywheel instance.
   *
   * @param canIds CAN IDs of the motors in the flywheel mechanism, in order from closest to
   *     furthest from the shooter.
   * @param reversed Whether each motor is reversed. First boolean corresponds to clockwise
   *     positive, / rest of the booleans correspond to whether each subsequent motor is reversed
   *     relative to the first motor.
   * @param gearRatio The gear ratio between the motor and the flywheel (motor rotations / output
   *     rotation).
   * @param currentLimit The current limit for the motors in amps.
   * @param canBus The CAN bus the motors are on, or an empty string for the rio bus.
   */
  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {
    public FlywheelHardwareConfig {
      MotorConfigValidation.validate(canIds, reversed, gearRatio, currentLimit, canBus);
      canIds = canIds.clone();
      reversed = reversed.clone();
    }

    @Override
    public int[] canIds() {
      return canIds.clone();
    }

    @Override
    public boolean[] reversed() {
      return reversed.clone();
    }
  }

  public record FlywheelSimulationConfig(double outputInertiaKgMetersSquared) {
    public FlywheelSimulationConfig {
      if (!Double.isFinite(outputInertiaKgMetersSquared) || outputInertiaKgMetersSquared <= 0)
        throw new IllegalArgumentException("Invalid inertia");
    }
  }

  public static final FlywheelSimulationConfig EXAMPLE_SIMULATION =
      new FlywheelSimulationConfig(0.025);

  /** Reference hardware config used as a template when adding new flywheels. */
  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {false}, 2.0, 40, "");

  /** Reference gains used as a template when adding new flywheels. */
  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.35, 0.065, 0.0, 1.0, 1.0);
}
