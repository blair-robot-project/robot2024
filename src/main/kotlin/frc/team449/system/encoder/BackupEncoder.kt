package frc.team449.system.encoder

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import kotlin.math.abs

/**
 * A wrapper to use when you have one external encoder that's more accurate but may be unplugged and
 * an integrated encoder that's less accurate but is less likely to be unplugged.
 *
 * <p>If the primary encoder's velocity is 0 but the integrated encoder's is above a given
 * threshold, it concludes that the primary encoder is broken and switches to using the
 * fallback/integrated encoder.
 */
class BackupEncoder(
  private val primary: Encoder,
  private val fallback: Encoder,
  private val velThreshold: Double,
  pollTime: Double = .02
) : Encoder(primary.name, 1, 1.0, 1.0, pollTime) {

  /** Whether the primary encoder's stopped working */
  private var useFallback = false

  override fun getPositionNative(): Double {
    return if (useFallback) {
      fallback.position
    } else {
      primary.position
    }
  }

  override fun pollVelocityNative(): Double {
    val fallbackVel = fallback.velocity
    return if (useFallback) {
      fallbackVel
    } else {
      val primaryVel = primary.velocity
      if (primaryVel == 0.0 && abs(fallbackVel) > velThreshold) {
        this.useFallback = true
        fallbackVel
      } else {
        primaryVel
      }
    }
  }

  companion object {
    fun <T : MotorController> creator(
      primaryCreator: EncoderCreator<T>,
      fallbackCreator: EncoderCreator<T>,
      velThreshold: Double
    ): EncoderCreator<T> = EncoderCreator { name, motor, inverted ->
      BackupEncoder(
        primaryCreator.create("primary_$name", motor, inverted),
        fallbackCreator.create("fallback_$name", motor, inverted),
        velThreshold
      )
    }
  }
}
