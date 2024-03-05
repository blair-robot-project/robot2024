package frc.team449.robot2024.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.subsystems.pivot.Pivot

class PivotCalibration(
  private val pivot: Pivot,
  private val numSamples: Int = 150
) : Command() {

  init {
    addRequirements(pivot)
  }

  private var samples = mutableListOf<Double>()

  override fun execute() {
    samples.add(pivot.motor.position)
  }

  override fun isFinished(): Boolean {
    return samples.size >= numSamples
  }

  override fun end(interrupted: Boolean) {
    samples.sort()
    val angle = samples[(samples.size * .9).toInt()]
    pivot.encoder.resetPosition(angle)
    println("***** Finished Calibrating Quadrature reading *****")
  }
}
