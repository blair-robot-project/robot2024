package frc.team449.robot2024.commands.light

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.system.light.Light

/** Description: Blink a certain color 5 times */
class PickupBlink {
  fun blinkGreen(light: Light): Command {
    val blinkCmd = InstantCommand({ setColor(light, 0, 255, 0) })
    val wait1Cmd = WaitCommand(0.15)
    val offCmd = InstantCommand({ setColor(light, 0, 0, 0) })
    val wait2Cmd = WaitCommand(0.10)

    return SequentialCommandGroup(
      blinkCmd,
      wait1Cmd,
      offCmd,
      wait2Cmd
    ).repeatedly()
  }

  private fun setColor(led: Light, r: Int, g: Int, b: Int) {
    for (i in 0 until led.buffer.length) {
      led.setRGB(i, r, g, b)
    }
  }
}
