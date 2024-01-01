package frc.team449.robot2024.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.constants.subsystem.LightConstants
import frc.team449.system.light.Light

class Rainbow(
  private val led: Light
) : Command() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstHue = 0.0

  override fun execute() {
    for (i in LightConstants.SECTION_1_START..LightConstants.SECTION_1_END) {
      val hue = MathUtil.inputModulus(firstHue + i * 180 / (LightConstants.SECTION_1_END - LightConstants.SECTION_1_START), 0.0, 180.0)

      led.setHSV(i, hue.toInt(), 255, 255)
    }

    for (i in LightConstants.SECTION_2_START..LightConstants.SECTION_2_END) {
      val hue = MathUtil.inputModulus(firstHue + (i - LightConstants.SECTION_2_START) * 180 / (LightConstants.SECTION_2_END - LightConstants.SECTION_2_START), 0.0, 180.0)

      led.setHSV(i, hue.toInt(), 255, 255)
    }

    firstHue += 6.15
    firstHue = MathUtil.inputModulus(firstHue, 0.0, 180.0)
  }
}
