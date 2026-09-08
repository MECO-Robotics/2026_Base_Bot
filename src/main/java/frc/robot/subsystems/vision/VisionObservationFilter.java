package frc.robot.subsystems.vision;

import static frc.robot.constants.vision.VisionConstants.*;

import java.util.Set;

/** The same validation is used for drivetrain fusion and QuestNav anchoring. */
public final class VisionObservationFilter {
  private VisionObservationFilter() {}

  public static boolean accepts(
      VisionIO.PoseObservation p, Set<Integer> allowed, boolean allowInertial) {
    if (p == null || p.pose() == null || p.type() == null) return false;
    var pose = p.pose();
    var q = pose.getRotation().getQuaternion();
    for (double v :
        new double[] {
          pose.getX(),
          pose.getY(),
          pose.getZ(),
          q.getW(),
          q.getX(),
          q.getY(),
          q.getZ(),
          p.timestamp(),
          p.ambiguity(),
          p.averageTagDistance()
        }) if (!Double.isFinite(v)) return false;
    if (p.timestamp() < 0
        || p.averageTagDistance() < 0
        || Math.abs(pose.getZ()) > maxZError
        || pose.getX() < 0
        || pose.getX() > aprilTagLayout.getFieldLength()
        || pose.getY() < 0
        || pose.getY() > aprilTagLayout.getFieldWidth()) return false;
    if (p.type() == VisionIO.PoseObservationType.QUESTNAV) return allowInertial;
    if (p.tagCount() < Math.max(1, minTagCountForOdometry)
        || (p.tagCount() == 1 && p.ambiguity() > maxAmbiguity)) return false;
    return allowed.isEmpty()
        || minWhitelistedTagCountForOdometry <= 0
        || p.tagIds().stream().filter(allowed::contains).count()
            >= minWhitelistedTagCountForOdometry;
  }
}
