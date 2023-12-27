package frc.team449.robot2023.subsystems.light

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.LightConstants

/**
 * Controls an LED strip.
 * @param port The PWM port of the LED strip.
 * @param length The length of the LED strip.
 */

class Light(
  port: Int,
  length: Int
) : SubsystemBase() {

  private val lightStrip = AddressableLED(port)
  val buffer = AddressableLEDBuffer(length)

  init {
    lightStrip.setLength(buffer.length)
    lightStrip.setData(buffer)
    lightStrip.start()
  }

  override fun periodic() {
    lightStrip.setData(buffer)
  }

  /** Copy the AddressableLEDBuffer setHsv() except to support
   * GRB LED strips */
  fun setHSV(index: Int, h: Int, s: Int, v: Int) {
    if (RobotBase.isReal()) {
      if (s == 0) {
        buffer.setRGB(index, v, v, v)
        return
      }

      // Difference between highest and lowest value of any rgb component
      val chroma = s * v / 255

      // Because hue is 0-180 rather than 0-360 use 30 not 60
      val region = h / 30 % 6

      // Remainder converted from 0-30 to 0-255
      val remainder = Math.round(h % 30 * (255 / 30.0)).toInt()

      // Value of the lowest rgb component
      val m = v - chroma

      // Goes from 0 to chroma as hue increases
      val X = chroma * remainder shr 8
      when (region) {
        0 -> buffer.setRGB(index, X + m, v, m)
        1 -> buffer.setRGB(index, v, v - X, m)
        2 -> buffer.setRGB(index, v, m, X + m)
        3 -> buffer.setRGB(index, v - X, m, v)
        4 -> buffer.setRGB(index, m, X + m, v)
        else -> buffer.setRGB(index, m, v, v - X)
      }
    } else {
      buffer.setHSV(index, h, s, v)
    }
  }

  /** Copy the AddressableLEDBuffer setRGB() except to support
   * GRB LED strips */
  fun setRGB(index: Int, r: Int, g: Int, b: Int) {
    if (RobotBase.isReal()) {
      buffer.setRGB(index, g, r, b)
    } else {
      buffer.setRGB(index, r, g, b)
    }
  }

  companion object {
    /** Create an LED strip controller using [LightConstants]. */
    fun createLight(): Light {
      return Light(
        LightConstants.LIGHT_PORT,
        LightConstants.LIGHT_LENGTH
      )
    }
  }
}
