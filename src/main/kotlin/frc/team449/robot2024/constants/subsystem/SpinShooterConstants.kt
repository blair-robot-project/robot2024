package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object SpinShooterConstants {
  const val EFFICIENCY = 0.8
  const val RIGHT_MOTOR_ID = 45
  const val RIGHT_MOTOR_INVERTED = true
  const val LEFT_MOTOR_ID = 46
  const val LEFT_MOTOR_INVERTED = false
  const val CURRENT_LIMIT = 50
  const val BRAKE_MODE = false

  val SUBWOOFER_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3850.0)
  val SUBWOOFER_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3850.0)
  val ANYWHERE_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4250.0)
  val ANYWHERE_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5750.0)
  val AUTO_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4750.0)
  val AUTO_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3650.0)
  val AMP_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2250.0)
  val OUTTAKE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(-200.0)

  const val AUTO_SHOOT_TOL = 30.0

  val BRAKE_RATE_LIMIT = Units.rotationsPerMinuteToRadiansPerSecond(3750.0)

  val SHOOTING_MAP = InterpolatingDoubleTreeMap()
  const val MAX_RANGE = 10.0 // m

  const val LEFT_KS = 0.051294
  const val RIGHT_KS = 0.067149
  const val LEFT_KV = 0.012064
  const val RIGHT_KV = 0.01155
  const val LEFT_KA = 0.011574
  const val RIGHT_KA = 0.0071641

  const val AMP_SCORE_VOLTAGE = 4.144
  const val DURING_INTAKE_VOLTAGE = -1.0

  /** In meters from the ground */
  const val SHOOTER_HEIGHT = 0.25

  const val IN_TOLERANCE = 6.5

  /** These constants are PER SIDE of the shooter */
  const val NUM_MOTORS = 1

  const val MODEL_VEL_STDDEV = 3.0
  const val INPT_ERR_STDDEV = 1e-6
  const val ENCODER_VEL_STDDEV = 0.01
  const val LQR_VEL_TOL = 15.0
  const val LQR_MAX_VOLTS = 12.0
  const val MAX_VOLTAGE = 12.0

  const val MIN_RAMP_VEL = 50.0

  /** Encoder stuff */
  const val INTERNAL_ENC_DEPTH = 4
  const val INTERNAL_MEASUREMENT_PD = 24
  const val UPR = 2 * PI
  const val GEARING = 2.0 / 1.0

  init {
    /**
     * Data is entered as follows:
     * SHOOTING_MAP.put(distanceToSpeaker, pivotAngle)
     */
    SHOOTING_MAP.put(1.25, Units.degreesToRadians(0.0))
    SHOOTING_MAP.put(1.5, Units.degreesToRadians(4.0))
    SHOOTING_MAP.put(1.75, Units.degreesToRadians(10.0))
    SHOOTING_MAP.put(2.25, Units.degreesToRadians(17.5))
    SHOOTING_MAP.put(3.0, Units.degreesToRadians(21.0))
    SHOOTING_MAP.put(7.0, Units.degreesToRadians(31.0))
  }
}
