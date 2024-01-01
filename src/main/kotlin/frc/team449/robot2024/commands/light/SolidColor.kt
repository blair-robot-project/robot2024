package frc.team449.robot2024.commands.light

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.system.light.Light

/** Description: Have a solid color  */
class SolidColor(
  private val led: Light,
  private val r: Int,
  private val g: Int,
  private val b: Int
) : Command() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      led.setRGB(i, r, g, b)
    }
  }
}
