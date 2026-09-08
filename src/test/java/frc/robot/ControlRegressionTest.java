package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.constants.types.FlywheelConstants;
import frc.robot.subsystems.flywheel.*;
import org.junit.jupiter.api.*;

class ControlRegressionTest {
  static int next;

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

  static class IO implements FlywheelIO {
    final String name = "ControlTest" + (next++);
    double requested, actual, voltage;

    public String getName() {
      return name;
    }

    public void setVelocity(double v) {
      requested = v;
    }

    public void setVoltage(double v) {
      voltage = v;
    }

    public void updateInputs(FlywheelIOInputs in) {
      in.velocity = actual;
      in.desiredVelocity = requested;
    }
  }

  @Test
  void defaultCommandReachesMotor() {
    var io = new IO();
    var f = new Flywheel(io, FlywheelConstants.EXAMPLE_GAINS);
    f.setDefaultCommand(f.run(() -> f.setVelocity(20)));
    for (int i = 0; i < 5; i++) CommandScheduler.getInstance().run();
    assertEquals(20, io.requested);
  }

  @Test
  void completedCommandRetainsTargetAndInterruptedCommandStops() {
    var io = new IO();
    var f = new Flywheel(io, FlywheelConstants.EXAMPLE_GAINS);
    var command = Flywheel.setVelocity(f, () -> 20);
    command.schedule();
    CommandScheduler.getInstance().run();
    assertTrue(command.isScheduled());
    io.actual = 20;
    CommandScheduler.getInstance().run();
    assertFalse(command.isScheduled());
    CommandScheduler.getInstance().run();
    assertEquals(20, io.requested);
    var next = Flywheel.setVelocity(f, () -> 30);
    next.schedule();
    next.cancel();
    f.periodic();
    assertEquals(0, io.requested);
  }

  @Test
  void staleIoTargetCannotFinishNewRequest() {
    var io = new IO();
    var f = new Flywheel(io, FlywheelConstants.EXAMPLE_GAINS);
    f.periodic();
    f.setVelocity(20);
    assertFalse(f.isFinished());
  }

  @Test
  void voltageCancellationStopsMotor() {
    var io = new IO();
    var f = new Flywheel(io, FlywheelConstants.EXAMPLE_GAINS);
    var c = new FlywheelVoltageCommand(f, () -> 4);
    c.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(4, io.voltage);
    c.cancel();
    assertEquals(0, io.voltage);
  }

  @Test
  void dashboardChangesOnlyApplyWhileUnowned() {
    var io = new IO();
    var f = new Flywheel(io, FlywheelConstants.EXAMPLE_GAINS);
    org.littletonrobotics.junction.Logger.AdvancedHooks.disableRobotBaseCheck();
    org.littletonrobotics.junction.Logger.start();
    try {
      var entry =
          edu.wpi.first.networktables.NetworkTableInstance.getDefault()
              .getEntry("/TunableNumbers/" + io.name + "/Gains/kSetpoint");
      entry.setDouble(12);
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser();
      f.periodic();
      assertEquals(12, io.requested);
      var c = f.run(() -> f.setVelocity(20));
      c.schedule();
      CommandScheduler.getInstance().run();
      entry.setDouble(40);
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser();
      CommandScheduler.getInstance().run();
      assertEquals(20, io.requested);
      c.cancel();
      f.periodic();
      assertEquals(20, io.requested);
      entry.setDouble(30);
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser();
      f.periodic();
      assertEquals(30, io.requested);
    } finally {
      org.littletonrobotics.junction.Logger.end();
    }
  }

  @Test
  void directGoalSurvivesInitialTunableRead() {
    var io = new IO();
    var f = new Flywheel(io, FlywheelConstants.EXAMPLE_GAINS);
    f.setVelocity(20);
    f.periodic();
    f.periodic();
    assertEquals(20, io.requested);
  }
}
