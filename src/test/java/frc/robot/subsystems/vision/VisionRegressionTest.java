package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.*;
import java.util.*;
import org.junit.jupiter.api.*;
import org.littletonrobotics.junction.LogTable;

class VisionRegressionTest {
  @BeforeAll
  static void hal() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  void reset() {
    VisionConstants.odometryTagWhitelist = Set.of();
    VisionConstants.minWhitelistedTagCountForOdometry = 0;
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  static PoseObservation pose(double time, Set<Integer> ids) {
    return new PoseObservation(
        time,
        new Pose3d(2, 2, 0, new Rotation3d()),
        0.1,
        1,
        2,
        PoseObservationType.PHOTONVISION,
        ids);
  }

  @Test
  void distinctFrameTagsCannotAuthorizeEachOther() {
    VisionConstants.odometryTagWhitelist = Set.of(1);
    VisionConstants.minWhitelistedTagCountForOdometry = 1;
    VisionIO io =
        new VisionIO() {
          public void updateInputs(VisionIOInputs in) {
            in.connected = true;
            in.tagIds = new int[] {1, 2};
            in.poseObservations = new PoseObservation[] {pose(1, Set.of(1)), pose(2, Set.of(2))};
          }
        };
    var accepted = new ArrayList<Double>();
    new Vision((p, t, d) -> accepted.add(t), io).periodic();
    assertEquals(List.of(1.0), accepted);
  }

  @Test
  void immutableDistinctIdsAndMissingMetadata() {
    Set<Integer> ids = new HashSet<>(List.of(1, 1));
    var p = pose(1, ids);
    ids.add(2);
    assertEquals(Set.of(1), p.tagIds());
    assertThrows(UnsupportedOperationException.class, () -> p.tagIds().add(3));
    VisionConstants.minWhitelistedTagCountForOdometry = 2;
    assertFalse(VisionObservationFilter.accepts(p, Set.of(1, 2), true));
    VisionConstants.minWhitelistedTagCountForOdometry = 1;
    assertFalse(VisionObservationFilter.accepts(pose(1, Set.of()), Set.of(1), true));
    assertTrue(VisionObservationFilter.accepts(pose(1, Set.of()), Set.of(), true));
  }

  @Test
  void malformedAndInertialObservations() {
    var valid = pose(1, Set.of(1));
    assertFalse(VisionObservationFilter.accepts(pose(Double.NaN, Set.of(1)), Set.of(), true));
    assertFalse(
        VisionObservationFilter.accepts(
            new PoseObservation(
                1,
                new Pose3d(Double.NaN, 2, 0, new Rotation3d()),
                0,
                1,
                1,
                PoseObservationType.PHOTONVISION,
                Set.of(1)),
            Set.of(),
            true));
    assertFalse(
        VisionObservationFilter.accepts(
            new PoseObservation(1, valid.pose(), Double.NaN, 1, 1, valid.type(), valid.tagIds()),
            Set.of(),
            true));
    assertFalse(
        VisionObservationFilter.accepts(
            new PoseObservation(
                1, valid.pose(), 0, 1, Double.POSITIVE_INFINITY, valid.type(), valid.tagIds()),
            Set.of(),
            true));
    VisionConstants.minWhitelistedTagCountForOdometry = 2;
    var inertial =
        new PoseObservation(1, valid.pose(), 0, -1, 0, PoseObservationType.QUESTNAV, Set.of());
    assertTrue(VisionObservationFilter.accepts(inertial, Set.of(1), true));
    assertFalse(VisionObservationFilter.accepts(inertial, Set.of(1), false));
  }

  @Test
  void loggingRoundTripAndLegacyWithoutTags() {
    var source = new VisionIOInputsLogged();
    source.connected = true;
    source.poseObservations = new PoseObservation[] {pose(1, Set.of(1, 2)), pose(2, Set.of(3))};
    var table = new LogTable(0);
    source.toLog(table);
    var read = new VisionIOInputsLogged();
    read.fromLog(table);
    assertArrayEquals(source.poseObservations, read.poseObservations);
    var legacy = new LogTable(0);
    var old =
        new VisionIOInputsLogged.Legacy.PoseObservation(
            1, pose(1, Set.of()).pose(), 0.1, 1, 2, PoseObservationType.PHOTONVISION);
    legacy.put("PoseObservations", new VisionIOInputsLogged.Legacy.PoseObservation[] {old});
    read.fromLog(legacy);
    assertEquals(1, read.poseObservations.length);
    assertTrue(read.poseObservations[0].tagIds().isEmpty());
    assertTrue(VisionObservationFilter.accepts(read.poseObservations[0], Set.of(), true));
  }

  @Test
  void limelightParsesEachFramesTagsAndLatency() {
    var nt =
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("test-limelight");
    var io = new VisionIOLimelight("test-limelight", new Transform3d());
    try (var pub = nt.getDoubleArrayTopic("botpose_wpiblue").publish()) {
      double[] packet = new double[] {2, 2, 0, 0, 0, 0, 10, 1, 0, 2, 0, 7, 0, 0, 0, 0, 0, 0.1};
      pub.set(packet);
      var in = new VisionIOInputs();
      io.updateInputs(in);
      assertEquals(1, in.poseObservations.length);
      assertEquals(Set.of(7), in.poseObservations[0].tagIds());
      assertEquals(0.1, in.poseObservations[0].ambiguity());
    }
  }
}
