package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.*;
import org.junit.jupiter.api.*;

class JointCommandRegressionTest {
  @BeforeAll
  static void hal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void enable() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void cleanup() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  static class IO implements PositionJointIO {
    double position, offset, requested, velocity, voltage;
    boolean brake = true;

    public String getName() {
      return "JointCommands";
    }

    public void updateInputs(PositionJointIOInputs in) {
      in.outputPosition = position - offset;
    }

    public void setPosition(double p) {
      requested = p;
      velocity = 2;
    }

    public boolean setPositionDynamic(double p, double v, double a) {
      requested = p;
      velocity = v;
      return true;
    }

    public void setVoltage(double v) {
      voltage = v;
    }

    public void setBrakeMode(boolean b) {
      brake = b;
    }

    public void resetPosition() {
      offset = position;
    }

    public double getPositionOffset() {
      return offset;
    }
  }

  @Test
  void interruptedConstraintsRestoreAndNewGoalsExitComplianceAndVoltage() {
    var io = new IO();
    var joint =
        new PositionJoint(io, new PositionJointGains(1, 0, 0, 0, 0, 0, 0, 2, 4, -1, 1, 0.01, 0));
    var command = PositionJoint.setPosition(joint, () -> 0.5, () -> 0.1, true);
    command.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(0.1, io.velocity);
    command.cancel();
    joint.periodic();
    assertEquals(2, io.velocity);
    assertEquals(0.5, io.requested);
    io.position = 0.5;
    joint.periodic();
    assertFalse(io.brake);
    joint.setPosition(0.7);
    joint.periodic();
    assertTrue(io.brake);
    assertEquals(0.7, io.requested);
    joint.setVoltage(3);
    joint.periodic();
    assertEquals(3, io.voltage);
    joint.setPosition(0.8, 0.2);
    joint.periodic();
    assertEquals(0.8, io.requested);
    joint.resetPosition();
    joint.periodic();
    assertEquals(0, joint.getPosition());
    assertEquals(0, io.requested);
    assertEquals(2, io.velocity);
    assertTrue(io.brake);
  }
}
