package frc.robot.sim;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.IdentityHashMap;

/** Accumulates one aggregate current per plant per loop, then sets next-loop battery voltage. */
public final class SimulationPower {
  private static final IdentityHashMap<Object, Double> currents = new IdentityHashMap<>();

  private SimulationPower() {}

  public static void beginCycle() {
    currents.clear();
  }

  public static void report(Object plant, double amps) {
    if (!Double.isFinite(amps)) throw new IllegalArgumentException("Nonfinite simulation current");
    currents.put(plant, Math.max(0, amps));
  }

  public static double totalCurrent() {
    return currents.values().stream().mapToDouble(Double::doubleValue).sum();
  }

  public static void endCycle() {
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent()));
  }
}
