package frc.team449.system.encoder

import edu.wpi.first.wpilibj.motorcontrol.MotorController

/** An external quadrature encoder */
class QuadEncoder(
  name: String,
  private val encoder: edu.wpi.first.wpilibj.Encoder,
  encoderCPR: Int,
  unitPerRotation: Double,
  gearing: Double,
  pollTime: Double = .01,
  samplesToAverage: Int = 5
) : Encoder(name, 1, 1.0, 1.0, pollTime) {
  init {
    // Let the WPI encoder handle the distance scaling
    encoder.distancePerPulse = unitPerRotation * gearing / encoderCPR
    encoder.samplesToAverage = samplesToAverage
  }

  override fun getPositionNative() = encoder.distance

  override fun pollVelocityNative() = encoder.rate

  companion object {
    fun <T : MotorController> creator(
      encoder: edu.wpi.first.wpilibj.Encoder,
      encoderCPR: Int,
      unitPerRotation: Double,
      gearing: Double,
      inverted: Boolean,
      samplesAverage: Int = 5
    ): EncoderCreator<T> =
      EncoderCreator { name, _, _ ->
        encoder.setReverseDirection(inverted)
        QuadEncoder(name, encoder, encoderCPR, unitPerRotation, gearing, samplesToAverage = samplesAverage)
      }
  }
}
