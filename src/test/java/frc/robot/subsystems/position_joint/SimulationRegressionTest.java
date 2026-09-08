package frc.robot.subsystems.position_joint;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.constants.types.PositionJointConstants.*;
import frc.robot.sim.SimulationPower;
import org.junit.jupiter.api.*;

class SimulationRegressionTest {
  @BeforeAll
  static void hal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @AfterEach
  void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12);
  }

  @Test
  void voltageDirectionAndDisabledCoastAgreeForBothVendors() throws Exception {
    for (boolean talon : new boolean[] {false, true})
      for (boolean inverted : new boolean[] {false, true}) {
        var config =
            new PositionJointHardwareConfig(
                new int[] {48, 49},
                new boolean[] {inverted, true},
                10,
                40,
                "",
                MechanismType.ROTATIONAL,
                GravityType.CONSTANT,
                0,
                EncoderCalibration.internal());
        var physical = new JointSimulationConfig(0.2, 1, 0.5, -2, 2, 0.25);
        try (JointControl io =
            talon
                ? new PositionJointIOSimTalonFX(
                    "direction", config, physical, DCMotor.getKrakenX60Foc(2))
                : new PositionJointIOSimSparkMax(
                    "direction", config, physical, DCMotor.getNEO(2))) {
          io.setGains(new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 1, 2, -2, 2, 0.01, 0.25));
          DriverStationSim.setDsAttached(true);
          DriverStationSim.setEnabled(true);
          DriverStationSim.notifyNewData();
          io.setVoltage(3);
          var in = new PositionJointIO.PositionJointIOInputs();
          for (int i = 0; i < 30; i++) {
            Unmanaged.feedEnable(100);
            Timer.delay(0.01);
            SimulationPower.beginCycle();
            io.updateInputs(in);
            for (double supply : in.motorSupplyVoltages) assertEquals(12, supply, 0.01);
            assertEquals(
                SimulationPower.totalCurrent(),
                java.util.Arrays.stream(in.motorCurrents).sum(),
                1e-5);
          }
          assertTrue(
              in.outputPosition > 0.25,
              "Positive voltage must move positive: Talon=" + talon + ", inverted=" + inverted);
          assertTrue(in.velocity > 0);
          double speed = in.velocity, position = in.outputPosition;
          io.setBrakeMode(false);
          DriverStationSim.setEnabled(false);
          DriverStationSim.notifyNewData();
          io.updateInputs(in);
          assertEquals(0, in.motorVoltages[0]);
          assertEquals(0, in.motorCurrents[0]);
          assertTrue(in.outputPosition > position);
          assertEquals(speed, in.velocity, 0.02);
        }
      }
  }
}
