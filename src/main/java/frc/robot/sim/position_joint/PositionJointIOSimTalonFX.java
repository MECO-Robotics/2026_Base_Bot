package frc.robot.sim.position_joint;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;

public class PositionJointIOSimTalonFX implements PositionJointIO {
  private final String name;
  private final PositionJointHardwareConfig config;
  private final DCMotorSim sim;
  private final PIDController controller;
  private final PositionJointFeedforward feedforward;
  private final boolean[] motorsConnected;
  private final double[] motorPositions;
  private final double[] motorVelocities;
  private final double[] motorVoltages;
  private final double[] motorCurrents;
  private double positionSetpoint = 0.0;
  private double velocitySetpoint = 0.0;

  public PositionJointIOSimTalonFX(
      String name, PositionJointHardwareConfig config, DCMotor simMotorModel) {
    this.name = name;
    this.config = config;
    motorsConnected = new boolean[config.canIds().length];
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(simMotorModel, 0.01, config.gearRatio()), simMotorModel);
    controller = new PIDController(0, 0, 0);
    feedforward = new TunableElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    double inputVoltage =
        controller.calculate(sim.getAngularPosition().in(Rotations), positionSetpoint)
            + feedforward.calculate(
                sim.getAngularPositionRotations(),
                sim.getAngularVelocity().in(RotationsPerSecond),
                velocitySetpoint,
                0.02);
    sim.setInputVoltage(inputVoltage);
    sim.update(0.02);

    inputs.outputPosition = sim.getAngularPosition().in(Rotations);
    inputs.desiredPosition = positionSetpoint;
    inputs.velocity = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < config.canIds().length; i++) {
      motorsConnected[i] = true;
      motorPositions[i] = sim.getAngularPosition().in(Rotations);
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);
      motorVoltages[i] = sim.getInputVoltage();
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorsConnected = motorsConnected;
    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;
    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setPosition(double position, double velocity) {
    positionSetpoint = position;
    velocitySetpoint = velocity;
  }

  @Override
  public void setGains(PositionJointGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kG(), gains.kV(), gains.kA());
  }

  @Override
  public String getName() {
    return name;
  }
}
