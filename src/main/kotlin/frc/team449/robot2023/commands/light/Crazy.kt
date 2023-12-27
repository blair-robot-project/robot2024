package frc.team449.robot2023.commands.light

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.subsystems.light.Light
import kotlin.random.Random

/** Description: Have a random color set for every led every 20ms
 *    a.k.a. you go blind  */
class Crazy(
  private val led: Light
) : Command() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      led.setRGB(i, Random.nextInt(0, 256), Random.nextInt(0, 256), Random.nextInt(0, 256))
    }
  }
}
