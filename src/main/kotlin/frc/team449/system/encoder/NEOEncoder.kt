package frc.team449.system.encoder

import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder

/** A NEO integrated encoder plugged into a Spark */
class NEOEncoder(
  name: String,
  private val enc: RelativeEncoder,
  unitPerRotation: Double,
  gearing: Double,
  pollTime: Double = .01,
  measurementPeriod: Int?,
  depth: Int?
) : Encoder(name, NEO_ENCODER_CPR, 1.0, 1.0, pollTime) {

  init {
    // Let the underlying encoder do the conversions
    enc.positionConversionFactor = unitPerRotation * gearing
    // Divide by 60 because it's originally in RPM
    enc.velocityConversionFactor = unitPerRotation * gearing / 60

    if (measurementPeriod != null) {
      enc.setMeasurementPeriod(measurementPeriod)
    }

    if (depth != null) {
      enc.setAverageDepth(depth)
    }
  }

  override fun getPositionNative() = enc.position

  override fun pollVelocityNative(): Double = enc.velocity

  companion object {
    const val NEO_ENCODER_CPR = 1

    fun creator(
      unitPerRotation: Double,
      gearing: Double,
      measurementPeriod: Int? = null,
      depth: Int? = null
    ): EncoderCreator<CANSparkMax> = EncoderCreator { name, motor, _ ->
      NEOEncoder(name, motor.encoder, unitPerRotation, gearing, measurementPeriod = measurementPeriod, depth = depth)
    }
  }
}
