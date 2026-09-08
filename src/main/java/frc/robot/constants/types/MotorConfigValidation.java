package frc.robot.constants.types;

import java.util.HashSet;

/** Shared validation before allocating motor controllers. */
public final class MotorConfigValidation {
  private MotorConfigValidation() {}

  public static void validate(
      int[] ids, boolean[] reversed, double gearing, int limit, String bus) {
    if (ids == null
        || reversed == null
        || ids.length == 0
        || ids.length != reversed.length
        || bus == null
        || !Double.isFinite(gearing)
        || gearing <= 0
        || limit <= 0) throw new IllegalArgumentException("Invalid motor configuration");
    var unique = new HashSet<Integer>();
    for (int id : ids)
      if (id < 0 || id > 62 || !unique.add(id))
        throw new IllegalArgumentException("Invalid or duplicate CAN ID");
  }
}
