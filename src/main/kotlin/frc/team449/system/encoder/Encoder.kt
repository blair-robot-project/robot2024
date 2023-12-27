package frc.team449.system.encoder

import edu.wpi.first.wpilibj.Timer

/**
 * A wrapper around encoders. Allows resetting encoders to a position.
 *
 * <p>Don't instantiate its subclasses directly. Instead, use their static creator methods
 * @param encoderCPR Counts per rotation of the encoder
 * @param unitPerRotation Meters traveled per rotation of the motor
 * @param gearing The factor the output changes by after being measured by the encoder
 *                (should be >= 1, not a reciprocal), e.g. this would be 70 if there were
 *                a 70:1 gearing between the encoder and the final output
 */
abstract class Encoder(
  val name: String,
  encoderCPR: Int,
  unitPerRotation: Double,
  gearing: Double,
  private val pollTime: Double
) {
  /**
   * Factor to multiply by to turn native encoder units into meters or whatever units are actually
   * wanted
   */
  private val encoderToUnit = unitPerRotation * gearing / encoderCPR

  /** An offset added to the position to allow resetting position. */
  private var positionOffset = 0.0

  /** Whether this encoder is being simulated */
  private var simulated = false

  /** Simulated position set by SimEncoderController */
  private var simPos = 0.0

  /** Simulated position set by SimEncoderController */
  private var simVel = 0.0

  /** Used to cache the velocity, because we want to update only when dt >= 20ms **/
  private var cachedVel = Double.NaN
  private var cachedTime = Double.NaN

  /** Current position in encoder's units */
  protected abstract fun getPositionNative(): Double

  /** Current velocity in encoder's units */
  protected abstract fun pollVelocityNative(): Double

  /** Position in meters or whatever unit you set */
  val position: Double
    get() {
      val posUnits = if (simulated) simPos else this.getPositionDirect()
      return positionOffset + posUnits
    }

  /** Velocity in meters per second or whatever unit you set */
  val velocity: Double
    get() {
      return if (simulated) simVel else getVelocityNative() * encoderToUnit
    }

  /**
   * Update the position offset to treat the current position as [pos]
   */
  open fun resetPosition(pos: Double) {
    this.positionOffset = pos - this.getPositionDirect()
  }

  /**
   * Get the position in units without adding [positionOffset]
   */
  private fun getPositionDirect() = this.getPositionNative() * encoderToUnit

  /**
   * Get the native velocity in units every loopTime
   */
  private fun getVelocityNative(): Double {
    val currTime = Timer.getFPGATimestamp()
    // update if it has been at least 20 ms since the last update
    if (cachedTime.isNaN() || currTime - cachedTime >= pollTime) {
      cachedVel = this.pollVelocityNative()
      cachedTime = currTime
    }
    return cachedVel
  }

  /**
   * Used to control [Encoder]s. Only one [SimController] can be used per encoder
   * object.
   *
   * @param enc The encoder to control.
   */
  class SimController(private val enc: Encoder) {
    init {
      if (enc.simulated) {
        throw IllegalStateException("${enc.name} is already being simulated.")
      }
      enc.simulated = true
    }

    /** Set the position of the [Encoder] this is controlling. */
    var position: Double
      get() = enc.simPos
      set(pos) {
        enc.simPos = pos
      }

    /** Set the velocity of the [Encoder] this is controlling. */
    var velocity: Double
      get() = enc.simVel
      set(vel) {
        enc.simVel = vel
      }
  }
}
