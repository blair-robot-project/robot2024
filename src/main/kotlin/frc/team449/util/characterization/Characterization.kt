// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.team449.util.characterization

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.LinkedList
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs

class Characterization(
  subsystem: Subsystem?,
  private val forward: Boolean,
  private val name: String,
  private val voltageConsumer: DoubleConsumer,
  private val velocitySupplier: DoubleSupplier
) : Command() {
  private val timer = Timer()
  private val data = FeedForwardCharacterizationData(name)

  init {
    addRequirements(subsystem)
  }


  // Called when the command is initially scheduled.
  override fun initialize() {
    timer.reset()
    timer.start()
  }

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
    if (timer.get() < startDelaySecs) {
      voltageConsumer.accept(0.0)
    } else {
      val voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * if (forward) 1 else -1
      voltageConsumer.accept(voltage)
      data.add(velocitySupplier.asDouble, voltage)
    }
  }

  // Called once the command ends or is interrupted.
  override fun end(interrupted: Boolean) {
    voltageConsumer.accept(0.0)

    timer.stop()
    println("hi i ended")
    if (!(data.velocityData.size == 0 || data.voltageData.size == 0)) {
      val regression = Regression(
        data.velocityData.stream().mapToDouble { obj: Double -> obj }.toArray(),
        data.voltageData.stream().mapToDouble { obj: Double -> obj }.toArray(),
        1
      )
      println("FF Characterization Results ($name):)\n\tCount= ${data.velocityData.size}\n\tR2=${regression.R2()}\n\tkS: ${regression.beta(0)}\n\tkV: ${regression.beta(1)}")
    }
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return false
  }

  class FeedForwardCharacterizationData(private val name: String) {
    val velocityData: MutableList<Double> = LinkedList()
    val voltageData: MutableList<Double> = LinkedList()
    fun add(velocity: Double, voltage: Double) {
      if (abs(velocity) > 1E-4) {
        velocityData.add(abs(velocity))
        voltageData.add(abs(voltage))
      }
    }
  }

  companion object {
    private const val startDelaySecs = 1.5
    private const val rampRateVoltsPerSec = 0.085
  }
}