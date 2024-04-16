package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import kotlin.math.PI
import kotlin.math.pow

object SpinShooterConstants {
  const val EFFICIENCY = 0.8
  const val RIGHT_MOTOR_ID = 45
  const val RIGHT_MOTOR_INVERTED = false
  const val LEFT_MOTOR_ID = 46
  const val LEFT_MOTOR_INVERTED = true
  const val CURRENT_LIMIT = 85
  const val SECONDARY_CURRENT_LIMIT = 200.0
  const val BRAKE_MODE = false

  val SUBWOOFER_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3450.0)
  val SUBWOOFER_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2800.0)
  val ANYWHERE_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4850.0)
  val ANYWHERE_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3850.0)
  val PASS_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4500.0)
  val PASS_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3500.0)
  val AMP_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(1800.0)
  val OUTTAKE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(-200.0)

  const val AUTO_SHOOT_TOL = 25.0

  val BRAKE_RATE_LIMIT = Units.rotationsPerMinuteToRadiansPerSecond(5250.0)

  val SHOOTING_MAP = InterpolatingDoubleTreeMap()
  val TIME_MAP = InterpolatingDoubleTreeMap()

  const val LEFT_KS = 0.28278 // 0.19599
  const val RIGHT_KS = 0.39721 // 0.28982
  const val LEFT_KV = 0.011015 // 0.010993
  const val RIGHT_KV = 0.011905 // 0.010836
  const val LEFT_KA = 0.0065218 // 0.0061217
  const val RIGHT_KA = 0.00714 // 0.00815

  const val IN_TOLERANCE = 15.0
  const val AIM_TOLERANCE = 19.5
  const val AMP_TOLERANCE = 75.0
  const val START_INPT_ERR = 50.0

  /** These constants are PER SIDE of the shooter */
  const val NUM_MOTORS = 1

  const val MODEL_VEL_STDDEV = 3.0
  const val INPT_ERR_STDDEV = 0.00215
  const val ENCODER_VEL_STDDEV = 0.01
  const val LQR_VEL_TOL = 2.75
  const val LQR_MAX_VOLTS = 12.0
  const val MAX_VOLTAGE = 12.0

  const val MIN_COAST_VEL = 15.0

  /** Encoder stuff */
  const val INTERNAL_ENC_DEPTH = 4
  const val INTERNAL_MEASUREMENT_PD = 24
  const val ENCODER_DELAY = 0.00875
  const val UPR = 2 * PI
  const val GEARING = 2.0 / 1.0

  val equation = { x: Double -> -78.7 + 2.18 * x - 0.0176 * x.pow(2) + 6.82e-5 * x.pow(3) - 1e-7 * x.pow(4) }

  init {
    /**
     * Data is entered as follows:
     * SHOOTING_MAP.put(distanceToSpeaker, pivotAngle)
     */
    SHOOTING_MAP.put(1.385103, 0.0)
    SHOOTING_MAP.put(2.140248, 0.232883)
    SHOOTING_MAP.put(2.749805, 0.345332)
    SHOOTING_MAP.put(3.291857, 0.384108)
    SHOOTING_MAP.put(4.148121, 0.447664)
    SHOOTING_MAP.put(4.935377, 0.514283)

    TIME_MAP.put(1.32, 0.20)
    TIME_MAP.put(2.29, 0.28)
    TIME_MAP.put(3.0, 0.375)
    TIME_MAP.put(3.73, 0.40)
    TIME_MAP.put(4.61, 0.50)
    TIME_MAP.put(5.24, 0.55)
  }
}
