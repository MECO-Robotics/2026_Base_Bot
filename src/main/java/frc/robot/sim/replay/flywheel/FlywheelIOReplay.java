package frc.robot.sim.replay.flywheel;

import frc.robot.subsystems.flywheel.FlywheelIO;

public class FlywheelIOReplay implements FlywheelIO {
  private final String name;

  public FlywheelIOReplay(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }
}
