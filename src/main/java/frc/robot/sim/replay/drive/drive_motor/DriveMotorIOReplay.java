package frc.robot.sim.replay.drive.drive_motor;

import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;

public class DriveMotorIOReplay implements DriveMotorIO {
  private final String name;

  public DriveMotorIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
