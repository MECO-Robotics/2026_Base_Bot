package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJointIO;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class ModeSelectionTest {
  @Test
  void modesConstructOnlyTheirSelectedSupplier() {
    var real = new AtomicInteger();
    var sim = new AtomicInteger();
    for (var mode : Mode.values()) {
      FlywheelIO.selectMode(
          mode,
          "F",
          () -> {
            real.incrementAndGet();
            return () -> "F";
          },
          () -> {
            sim.incrementAndGet();
            return () -> "F";
          });
      PositionJointIO.selectMode(
          mode,
          "J",
          () -> {
            real.incrementAndGet();
            return () -> "J";
          },
          () -> {
            sim.incrementAndGet();
            return () -> "J";
          });
    }
    assertEquals(2, real.get());
    assertEquals(2, sim.get());
  }
}
