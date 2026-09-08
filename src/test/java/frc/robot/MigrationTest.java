package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.types.PositionJointConstants;
import frc.robot.subsystems.position_joint.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.UnitInterpolatingMap;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class MigrationTest {
  private static final PositionJointConstants.PositionJointGains TEST_GAINS =
      new PositionJointConstants.PositionJointGains(
          1.5, 0, 0, 0.5, 1, 2, 0, 10, 20, 0, Math.PI, 0.2, 0);

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  static class JointIO implements PositionJointIO {
    double measured;
    double goal;
    double volts;
    boolean brake = true;
    int positionCalls;

    public String getName() {
      return "MigrationJoint";
    }

    public void updateInputs(PositionJointIOInputs inputs) {
      inputs.outputPosition = measured;
    }

    public void setPosition(double position) {
      goal = position;
      positionCalls++;
    }

    public void setVoltage(double voltage) {
      volts = voltage;
    }

    public void setBrakeMode(boolean enabled) {
      brake = enabled;
    }
  }

  @Test
  void jointRetainsGoalAndOpenLoopOwnership() {
    JointIO io = new JointIO();
    PositionJoint joint = new PositionJoint(io, TEST_GAINS);
    joint.periodic();
    joint.setPosition(1.0);
    joint.periodic();
    joint.periodic();
    assertEquals(1.0, io.goal);
    joint.setVoltage(3.0);
    int calls = io.positionCalls;
    joint.periodic();
    assertEquals(calls, io.positionCalls);
    assertEquals(3.0, io.volts);
    joint.setPosition(2.0);
    joint.periodic();
    assertEquals(2.0, io.goal);
  }

  @Test
  void complianceStaysReleasedUntilNewGoal() {
    JointIO io = new JointIO();
    PositionJoint joint = new PositionJoint(io, TEST_GAINS);
    joint.periodic();
    joint.setPosition(1.0);
    joint.setComplianceAfterTarget(true);
    io.measured = 1.0;
    joint.periodic();
    assertFalse(io.brake);
    assertEquals(0.0, io.volts);
    int calls = io.positionCalls;
    io.measured = 0.5;
    joint.periodic();
    assertEquals(calls, io.positionCalls);
    joint.setPosition(2.0);
    joint.periodic();
    assertTrue(io.brake);
    assertEquals(2.0, io.goal);
  }

  @Test
  void visionAcceptsGoodSingleTagAndInertialPoseButRejectsAmbiguity() {
    var accepted = new ArrayList<Double>();
    VisionIO io =
        new VisionIO() {
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.poseObservations =
                new PoseObservation[] {
                  new PoseObservation(
                      1.0,
                      new Pose3d(2, 2, 0, new Rotation3d()),
                      0.1,
                      1,
                      2,
                      PoseObservationType.PHOTONVISION,
                      java.util.Set.of(1)),
                  new PoseObservation(
                      2.0,
                      new Pose3d(2, 2, 0, new Rotation3d()),
                      0.9,
                      1,
                      2,
                      PoseObservationType.PHOTONVISION,
                      java.util.Set.of(1)),
                  new PoseObservation(
                      3.0,
                      new Pose3d(2, 2, 0, new Rotation3d()),
                      0,
                      0,
                      0,
                      PoseObservationType.QUESTNAV,
                      java.util.Set.of())
                };
          }
        };
    new Vision(
            (pose, timestamp, deviations) -> {
              accepted.add(timestamp);
              assertTrue(Double.isFinite(deviations.get(0, 0)));
            },
            io)
        .periodic();
    assertEquals(java.util.List.of(1.0, 3.0), accepted);
  }

  @Test
  void interpolationConvertsUnitsAndClampsEndpoints() {
    var map = new UnitInterpolatingMap<>(Meters, RadiansPerSecond);
    assertEquals(0, map.get(Meters.of(1)).in(RadiansPerSecond));
    map.put(Meters.of(1), RadiansPerSecond.of(10));
    map.put(Meters.of(3), RadiansPerSecond.of(30));
    assertEquals(20, map.get(Centimeters.of(200)).in(RadiansPerSecond), 1e-9);
    assertEquals(10, map.get(Meters.of(-1)).in(RadiansPerSecond));
    assertEquals(30, map.get(Meters.of(5)).in(RadiansPerSecond));
  }
}
