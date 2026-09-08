package frc.robot.subsystems.position_joint;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.types.PositionJointConstants.*;
import frc.robot.sim.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.*;

class JointRegressionTest {
  static int nextId = 20;

  @BeforeAll
  static void hal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @AfterEach
  void cleanup() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12);
  }

  static PositionJointGains gains() {
    return new PositionJointGains(4, 0, 0, 0, 1, 2, 0, 10, 20, -2, 2, 0.01, 0);
  }

  static PositionJointHardwareConfig config(EncoderType sensor, GravityType gravity) {
    return new PositionJointHardwareConfig(
        new int[] {nextId++},
        new boolean[] {false},
        10,
        30,
        "",
        MechanismType.ROTATIONAL,
        gravity,
        0,
        new EncoderCalibration(sensor, 0, 1, 0, false));
  }

  static class Spark extends PositionJointIOSparkMax {
    Spark(PositionJointHardwareConfig c, java.util.function.DoubleSupplier ff) {
      super("SparkTest", c, ff, true);
    }

    SparkMax motor() {
      return motors[0];
    }
  }

  static class Talon extends PositionJointIOTalonFX {
    Talon(PositionJointHardwareConfig c) {
      super("TalonTest", c);
    }

    TalonFX motor() {
      return motors[0];
    }
  }

  @Test
  void sparkVelocityAndExternalFeedforwardAndConstraints() throws Exception {
    AtomicInteger calls = new AtomicInteger();
    try (var io =
        new Spark(
            config(EncoderType.INTERNAL, GravityType.COSINE),
            () -> {
              calls.incrementAndGet();
              return 2;
            })) {
      io.setGains(gains());
      assertEquals(2, io.motor().configAccessor.closedLoop.feedForward.getkV(), 1e-6);
      io.setPositionDynamic(1, 0.5, 2);
      assertEquals(0.5, io.motor().configAccessor.closedLoop.maxMotion.getCruiseVelocity(), 1e-6);
      io.setPosition(1);
      assertEquals(10, io.motor().configAccessor.closedLoop.maxMotion.getCruiseVelocity(), 1e-6);
      assertEquals(2, calls.get());
    }
  }

  @Test
  void sparkAbsoluteFeedbackMatchesTelemetryAndReset() throws Exception {
    try (var io = new Spark(config(EncoderType.EXTERNAL_SPARK, GravityType.COSINE), () -> 0)) {
      io.setGains(gains());
      var sim = new SparkMaxSim(io.motor(), DCMotor.getNEO(1));
      sim.getRelativeEncoderSim().setPosition(0.1);
      sim.getAbsoluteEncoderSim().setPosition(0.3);
      var in = new PositionJointIO.PositionJointIOInputs();
      io.updateInputs(in);
      assertEquals(0.3, in.outputPosition, 1e-6);
      assertEquals(1.0, in.rotorPosition, 1e-6);
      io.resetPosition();
      io.updateInputs(in);
      assertEquals(0, in.outputPosition, 1e-6);
      assertEquals(0.3, io.motor().getAbsoluteEncoder().getPosition(), 1e-6);
      io.setPosition(0.2);
      assertEquals(0.5, sim.getSetpoint(), 1e-6);
    }
  }

  @Test
  void talonGravityAndDynamicRestoration() throws Exception {
    try (var io = new Talon(config(EncoderType.INTERNAL, GravityType.CONSTANT))) {
      io.setGains(gains());
      var slot = new Slot0Configs();
      assertTrue(io.motor().getConfigurator().refresh(slot).isOK());
      assertEquals(1, slot.kG);
      io.setPositionDynamic(1, 0.5, 2);
      io.setPosition(1);
      var motion = new MotionMagicConfigs();
      assertTrue(io.motor().getConfigurator().refresh(motion).isOK());
      assertEquals(10, motion.MotionMagicCruiseVelocity);
    }
  }

  @Test
  void sineAndCosineUseRotations() {
    var sine = config(EncoderType.INTERNAL, GravityType.SINE);
    var cos = config(EncoderType.INTERNAL, GravityType.COSINE);
    assertEquals(0, sine.gravityVoltage(0, 2), 1e-9);
    assertEquals(2, sine.gravityVoltage(0.25, 2), 1e-9);
    assertEquals(0, cos.gravityVoltage(0.25, 2), 1e-9);
    assertEquals(0, PositionJointIOTalonFX.gainConfiguration(sine, gains()).kG);
  }

  @Test
  void vendorSimResetKeepsPhysicalState() throws Exception {
    var physical = new JointSimulationConfig(0.1, 2, 0.5, -2, 2, 0.25);
    var c = config(EncoderType.INTERNAL, GravityType.CONSTANT);
    try (var io = new PositionJointIOSimSparkMax("SparkSim", c, physical, DCMotor.getNEO(1))) {
      io.setGains(gains());
      io.resetPosition();
      assertEquals(0.25, io.physicalPositionForSimulation(), 1e-9);
      assertEquals(0.25, io.getPositionOffset(), 1e-9);
    }
    c = config(EncoderType.INTERNAL, GravityType.CONSTANT);
    try (var io =
        new PositionJointIOSimTalonFX("TalonSim", c, physical, DCMotor.getKrakenX60Foc(1))) {
      io.setGains(gains());
      io.resetPosition();
      assertEquals(0.25, io.physicalPositionForSimulation(), 1e-9);
      assertEquals(0.25, io.getPositionOffset(), 1e-9);
    }
  }

  @Test
  void linearTravelAndGravityAndPhysicalLimitValidation() {
    var c =
        new PositionJointHardwareConfig(
            new int[] {2},
            new boolean[] {false},
            5,
            30,
            "",
            MechanismType.LINEAR,
            GravityType.CONSTANT,
            0.02,
            EncoderCalibration.internal());
    assertEquals(5 / (2 * Math.PI * 0.02), c.motorRotationsPerUnit(), 1e-9);
    var p =
        new JointPhysics(c, new JointSimulationConfig(0.1, 5, 0.5, 0, 2, 1.5), DCMotor.getNEO(1));
    assertEquals(1.5, p.position());
    assertThrows(IllegalArgumentException.class, () -> p.validateLimits(0, 3));
    p.step(0, false, false);
    assertTrue(p.position() < 1.5, "coasting elevator must fall under gravity");
    assertEquals(0, p.current());
  }

  @Test
  void nativeSparkFeedforwardIsAppliedOnceAndLimitsUseAbsoluteFeedback() throws Exception {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    try (var io = new Spark(config(EncoderType.EXTERNAL_SPARK, GravityType.CONSTANT), () -> 2)) {
      io.setGains(new PositionJointGains(0, 0, 0, 0, 1, 0, 0, 1, 2, 0, 0.9, 0.01, 0.3));
      var sim = new SparkMaxSim(io.motor(), DCMotor.getNEO(1));
      sim.setPosition(0.3);
      sim.getAbsoluteEncoderSim().setPosition(0.3);
      io.setPosition(0.3);
      // Settle the native current limiter independently of the stationary sensor fixture.
      for (int i = 0; i < 1000; i++) {
        sim.setMotorCurrent(0);
        sim.iterate(0, 12, 0.02);
      }
      assertEquals(3, sim.getAppliedOutput() * 12, 0.01);
      io.setPositionDynamic(0.3, 0.5, 1);
      // Settle the native current limiter independently of the stationary sensor fixture.
      for (int i = 0; i < 1000; i++) {
        sim.setMotorCurrent(0);
        sim.iterate(0, 12, 0.02);
      }
      assertEquals(3, sim.getAppliedOutput() * 12, 0.01);
      sim.setPosition(0.95);
      sim.getAbsoluteEncoderSim().setPosition(0.95);
      sim.getRelativeEncoderSim().setPosition(0);
      io.setVoltage(4);
      sim.iterate(0, 12, 0.02);
      assertEquals(0, sim.getAppliedOutput(), 1e-6);
    }
  }

  @Test
  void disabledPlantFallsBrakeDampsAndResetPreservesVelocity() throws Exception {
    var c = config(EncoderType.INTERNAL, GravityType.COSINE);
    var physical = new JointSimulationConfig(0.1, 2, 0.5, -2, 2, 0);
    var coast = new JointPhysics(c, physical, DCMotor.getNEO(1));
    var brake = new JointPhysics(c, physical, DCMotor.getNEO(1));
    for (int i = 0; i < 15; i++) {
      coast.step(0, false, false);
      brake.step(0, true, false);
    }
    assertTrue(coast.position() < 0);
    assertTrue(brake.position() < 0);
    assertTrue(Math.abs(coast.velocity()) > Math.abs(brake.velocity()));
    try (var io = new PositionJointIOSimSparkMax("fall", c, physical, DCMotor.getNEO(1))) {
      io.setGains(gains());
      io.setBrakeMode(false);
      io.setVoltage(0);
      var in = new PositionJointIO.PositionJointIOInputs();
      for (int i = 0; i < 5; i++) io.updateInputs(in);
      double before = io.physicalPositionForSimulation(),
          velocity = io.physicalVelocityForSimulation();
      io.resetPosition();
      assertEquals(before, io.physicalPositionForSimulation());
      assertEquals(velocity, io.physicalVelocityForSimulation());
      assertEquals(before, io.getPositionOffset());
    }
  }

  @Test
  void followersReceiveCurrentLimitsAndNativeLeaderRequests() throws Exception {
    var c =
        new PositionJointHardwareConfig(
            new int[] {nextId++, nextId++},
            new boolean[] {false, true},
            10,
            27,
            "",
            MechanismType.ROTATIONAL,
            GravityType.CONSTANT,
            0,
            EncoderCalibration.internal());
    try (var io = new Talon(c)) {
      var limits = new CurrentLimitsConfigs();
      assertTrue(io.motors[1].getConfigurator().refresh(limits).isOK());
      assertEquals(27, limits.SupplyCurrentLimit);
      assertTrue(limits.SupplyCurrentLimitEnable);
      var request = io.motors[1].getAppliedControl().getControlInfo();
      assertEquals(Integer.toString(c.canIds()[0]), request.get("LeaderID"));
    }
    c =
        new PositionJointHardwareConfig(
            new int[] {nextId++, nextId++},
            new boolean[] {false, true},
            10,
            27,
            "",
            MechanismType.ROTATIONAL,
            GravityType.CONSTANT,
            0,
            EncoderCalibration.internal());
    try (var io = new Spark(c, () -> 0)) {
      assertEquals(27, io.motors[1].configAccessor.getSmartCurrentLimit());
      assertEquals(c.canIds()[0], io.motors[1].configAccessor.getFollowerModeLeaderId());
      var sim = new SparkMaxSim(io.motor(), DCMotor.getNEO(1));
      sim.setBusVoltage(9);
      sim.setAppliedOutput(0.5);
      var in = new PositionJointIO.PositionJointIOInputs();
      io.updateInputs(in);
      assertEquals(4.5, in.motorVoltages[0], 1e-6);
      assertEquals(9, in.motorSupplyVoltages[0], 1e-6);
      assertTrue(in.motorsConnected[0]);
    }
  }

  @Test
  void badConfigurationsFailBeforeHardwareAllocation() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new PositionJointHardwareConfig(
                new int[] {1, 1},
                new boolean[] {false, false},
                10,
                30,
                "",
                MechanismType.ROTATIONAL,
                GravityType.COSINE,
                0,
                EncoderCalibration.internal()));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new PositionJointHardwareConfig(
                new int[] {1},
                new boolean[] {false},
                0,
                30,
                "",
                MechanismType.ROTATIONAL,
                GravityType.COSINE,
                0,
                EncoderCalibration.internal()));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new PositionJointHardwareConfig(
                new int[] {1},
                new boolean[] {false},
                1,
                30,
                "",
                MechanismType.LINEAR,
                GravityType.CONSTANT,
                0,
                EncoderCalibration.internal()));
  }

  @Test
  void calibrationAndPowerAreDeterministic() {
    var e = new EncoderCalibration(EncoderType.EXTERNAL_SPARK, 0, 2, 0.1, true);
    assertEquals(-0.4, e.calibrate(0.3), 1e-9);
    Object a = new Object(), b = new Object();
    SimulationPower.beginCycle();
    SimulationPower.report(a, 10);
    SimulationPower.report(b, 20);
    SimulationPower.report(a, 12);
    assertEquals(32, SimulationPower.totalCurrent());
    SimulationPower.endCycle();
    double volts = edu.wpi.first.wpilibj.RobotController.getBatteryVoltage();
    SimulationPower.beginCycle();
    SimulationPower.report(b, 20);
    SimulationPower.report(a, 12);
    SimulationPower.endCycle();
    assertEquals(volts, edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
  }
}
