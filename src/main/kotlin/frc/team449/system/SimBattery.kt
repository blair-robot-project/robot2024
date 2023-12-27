package frc.team449.system

import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim

/**
 * Used for simulating battery voltage and current based off drawn currents from other mechanisms
 */
class SimBattery {
  private val currentDrawers = mutableListOf<() -> Double>()

  /**
   * Register a simulated mechanism drawing current
   */
  fun addCurrentDrawer(currentDrawer: () -> Double) {
    currentDrawers.add(currentDrawer)
  }

  /**
   * Update the simulation with a newly calculated battery current and voltage
   */
  fun update() {
    val currents = currentDrawers.map { it() }
    RoboRioSim.setVInCurrent(currents.sum())
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(*currents.toDoubleArray()))
  }
}
