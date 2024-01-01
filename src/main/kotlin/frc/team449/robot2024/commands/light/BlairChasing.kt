package frc.team449.robot2024.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.constants.subsystem.LightConstants
import frc.team449.system.light.Light

/** Description: Have a linear transition from white to red */
class BlairChasing(
  private val led: Light
) : Command() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstSaturation1 = 0.0
  private var firstSaturation2 = 0.0

  override fun execute() {
    for (i in LightConstants.SECTION_1_START..LightConstants.SECTION_1_END) {
      // This number is related to how many lights will show up between the high and low intensity (which technically also affects how fast itll cycle)
      val saturation = MathUtil.inputModulus(firstSaturation1 + i * 200.0 / led.buffer.length, 0.0, 255.0)
      led.setHSV(i, 0, saturation.toInt(), 255)

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstSaturation1 += 0.135
      firstSaturation1 = MathUtil.inputModulus(firstSaturation1, 0.0, 255.0)
    }

    for (i in LightConstants.SECTION_2_START..LightConstants.SECTION_2_END) {
      // This number is related to how many lights will show up between the high and low intensity (which technically also affects how fast itll cycle)
      val saturation = MathUtil.inputModulus(firstSaturation2 + (i - LightConstants.SECTION_2_START) * 200.0 / led.buffer.length, 0.0, 255.0)
      led.setHSV(i, 0, saturation.toInt(), 255)

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstSaturation2 += 0.135
      firstSaturation2 = MathUtil.inputModulus(firstSaturation2, 0.0, 255.0)
    }
  }
}
