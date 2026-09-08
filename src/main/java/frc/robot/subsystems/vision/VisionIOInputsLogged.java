package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.*;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Keeps the legacy PoseObservation struct schema for old replay files; IDs are an added column. */
public final class VisionIOInputsLogged extends VisionIO.VisionIOInputs implements LoggableInputs {
  public static final class Legacy {
    public record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        VisionIO.PoseObservationType type) {}
  }

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("LatestTargetObservation", latestTargetObservation);
    table.put("TagIds", tagIds);
    var legacy = new Legacy.PoseObservation[poseObservations.length];
    for (int i = 0; i < legacy.length; i++) {
      var p = poseObservations[i];
      legacy[i] =
          new Legacy.PoseObservation(
              p.timestamp(),
              p.pose(),
              p.ambiguity(),
              p.tagCount(),
              p.averageTagDistance(),
              p.type());
      table.put(
          "ObservationTags/" + i,
          p.tagIds().stream().sorted().mapToInt(Integer::intValue).toArray());
    }
    table.put("PoseObservations", legacy);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", false);
    latestTargetObservation = table.get("LatestTargetObservation", latestTargetObservation);
    tagIds = table.get("TagIds", new int[0]);
    var legacy = table.get("PoseObservations", new Legacy.PoseObservation[0]);
    poseObservations = new VisionIO.PoseObservation[legacy.length];
    for (int i = 0; i < legacy.length; i++) {
      var p = legacy[i];
      var ids = new HashSet<Integer>();
      for (int id : table.get("ObservationTags/" + i, new int[0])) ids.add(id);
      poseObservations[i] =
          new VisionIO.PoseObservation(
              p.timestamp(),
              p.pose(),
              p.ambiguity(),
              p.tagCount(),
              p.averageTagDistance(),
              p.type(),
              ids);
    }
  }
}
