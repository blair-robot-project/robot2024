package frc.team449.robot2023.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.constants.subsystem.LightConstants
import frc.team449.robot2023.subsystems.light.Light

/** Description: Have a linear transition from a darker to brighter specified hue */
class BreatheHue(
  private val led: Light,
  private val hue: Int
) : Command() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstIntensity1 = 175.0
  private var firstIntensity2 = 175.0


  override fun execute() {
    for (i in LightConstants.SECTION_1_START..LightConstants.SECTION_1_END) {
      // This number is related to how many lights will show up between the high and low intensity
      val intensity = MathUtil.inputModulus(firstIntensity1 + i * 32.5 / led.buffer.length, 125.0, 255.0)
      led.setHSV(i, hue, 255, intensity.toInt())

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstIntensity1 += 0.1
      firstIntensity1 = MathUtil.inputModulus(firstIntensity1, 125.0, 255.0)
    }

    for (i in LightConstants.SECTION_2_START..LightConstants.SECTION_2_END) {
      // This number is related to how many lights will show up between the high and low intensity
      val intensity = MathUtil.inputModulus(firstIntensity2 + (i - LightConstants.SECTION_2_START) * 32.5 / led.buffer.length, 125.0, 255.0)
      led.setHSV(i, hue, 255, intensity.toInt())

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstIntensity2 += 0.1
      firstIntensity2 = MathUtil.inputModulus(firstIntensity2, 125.0, 255.0)
    }
  }
}
