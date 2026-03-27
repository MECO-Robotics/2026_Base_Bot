package frc.robot.sim.replay.piece_detection;

import frc.robot.subsystems.piece_detection.PieceDetectionIO;

public class PieceDetectionIOReplay implements PieceDetectionIO {
  private final String name;

  public PieceDetectionIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
