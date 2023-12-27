package frc.team449.system.encoder

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.motorcontrol.MotorController

/**
 * This class uses an absolute encoder, gear ratio and UPR to give the absolute position of the module or rotational velocity of the module.
 *
 * @param offset This must be in rotations of how much the offset of the ENCODER should be.
 */
open class AbsoluteEncoder(
  name: String,
  private val enc: DutyCycleEncoder,
  unitPerRotation: Double,
  private val inverted: Boolean,
  private var offset: Double,
  pollTime: Double = .02,
  samplesPerAverage: Int = 1
) : Encoder(name, 1, unitPerRotation, 1.0, pollTime) {
  private var prevPos = Double.NaN
  private var prevTime = Double.NaN
  private val filter = MedianFilter(samplesPerAverage)

  private val frequency = enc.frequency

  /** This returns the absolute position of the module */
  override fun getPositionNative(): Double {
    return if (inverted) {
      filter.calculate(
        MathUtil.inputModulus(
          1 - (enc.absolutePosition - offset),
          -0.5,
          0.5
        )
      )
    } else {
      filter.calculate(
        MathUtil.inputModulus(
          (enc.absolutePosition - offset),
          -0.5,
          0.5
        )

      )
    }
  }

  override fun resetPosition(pos: Double) {
    offset += getPositionNative() - pos
  }

  /** This returns the rotational velocity (on vertical axis) of the module */
  override fun pollVelocityNative(): Double {
    val currPos =
      if (inverted) {
        -enc.distance
      } else {
        enc.distance
      }

    val currTime = Timer.getFPGATimestamp()

    val vel =
      if (prevPos.isNaN()) {
        0.0
      } else {
        val dt = currTime - prevTime
        val dx = currPos - prevPos
        dx / dt
      }
    this.prevPos = currPos
    this.prevTime = currTime

    return vel
  }

  companion object {
    /**
     * @param <T>
     * @param channel The DutyCycleEncoder port
     * @param offset The position to put into DutyCycleEncoder's setPositionOffset
     * @param unitPerRotation units measured when done one rotation (e.g 360 degrees per rotation)
     * @param inverted If the encoder needs to be inverted or not
     */
    fun <T : MotorController> creator(
      channel: Int,
      offset: Double,
      unitPerRotation: Double,
      inverted: Boolean
    ): EncoderCreator<T> =
      EncoderCreator { name, _, _ ->
        val enc = AbsoluteEncoder(
          name,
          DutyCycleEncoder(channel),
          unitPerRotation,
          inverted,
          offset
        )
        enc
      }
  }
}
