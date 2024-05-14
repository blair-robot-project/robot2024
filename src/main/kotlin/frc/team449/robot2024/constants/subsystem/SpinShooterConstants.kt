package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import kotlin.math.PI
import kotlin.math.pow

object SpinShooterConstants {
  val BRAKE_MODE = false
  const val EFFICIENCY = 0.8

  const val DUTY_CYCLE_DEADBAND = 0.001
  const val RIGHT_MOTOR_ID = 45
  const val RIGHT_MOTOR_INVERTED = false
  val RIGHT_MOTOR_ORIENTATION = InvertedValue.CounterClockwise_Positive
  val RIGHT_NEUTRAL_MODE = NeutralModeValue.Coast
  const val LEFT_MOTOR_ID = 46
  const val LEFT_MOTOR_INVERTED = true
  val LEFT_MOTOR_ORIENTATION = InvertedValue.CounterClockwise_Positive
  val LEFT_NEUTRAL_MODE = NeutralModeValue.Coast

  const val UPDATE_FREQUENCY = 100.0

  const val STATOR_CURRENT_LIMIT = 150.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val BURST_CURRENT_LIMIT = 60.0
  const val BURST_TIME_LIMIT = 0.25
  const val SECONDARY_CURRENT_LIMIT = 200.0

  val SUBWOOFER_LEFT_SPEED = 3450.0 / 60
  val SUBWOOFER_RIGHT_SPEED = 2800.0 / 60
  val ANYWHERE_LEFT_SPEED = 4850.0 / 60
  val ANYWHERE_RIGHT_SPEED = 3850.0 / 60
  val PASS_LEFT_SPEED = 3900.0 / 60
  val PASS_RIGHT_SPEED = 2900.0 / 60
  val PASS2_LEFT_SPEED = 3550.0 / 60
  val PASS2_RIGHT_SPEED = 2550.0 / 60
  val PASS3_LEFT_SPEED = 3550.0 / 60
  val PASS3_RIGHT_SPEED = 2550.0 / 60
  val AMP_SPEED = 1800.0 / 60
  val OUTTAKE_SPEED = -200.0 / 60

  const val AUTO_SHOOT_TOL = 25.0

  val BRAKE_RATE_LIMIT = 5250.0 / 60

  val SHOOTING_MAP = InterpolatingDoubleTreeMap()
  val TIME_MAP = InterpolatingDoubleTreeMap()

  const val LEFT_KS = 0.2377 // 0.19599
  const val RIGHT_KS = 0.32957 // 0.28982
  const val LEFT_KV = 0.010607 // 0.010993
  const val RIGHT_KV = 0.011597 // 0.010836
  const val LEFT_KA = 0.0050 // 0.0061217
  const val RIGHT_KA = 0.0060 // 0.00815
  const val LEFT_KP = 0.2377 // 0.19599
  const val RIGHT_KP = 0.32957 // 0.28982
  const val LEFT_KI = 0.010607 // 0.010993
  const val RIGHT_KI = 0.011597 // 0.010836
  const val LEFT_KD = 0.0050 // 0.0061217
  const val RIGHT_KD = 0.0060 // 0.00815

  const val IN_TOLERANCE = 10.0
  const val AIM_TOLERANCE = 15.0
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
  const val GEARING = 1.0 / 2.0

  val equation = { x: Double -> -78.7 + 2.18 * x - 0.0176 * x.pow(2) + 6.82e-5 * x.pow(3) - 1e-7 * x.pow(4) }

  init {
    /**
     * Data is entered as follows:
     * SHOOTING_MAP.put(distanceToSpeaker, pivotAngle)
     */
    SHOOTING_MAP.put(1.385103, 0.0)
    SHOOTING_MAP.put(2.202962, 0.242588)
    SHOOTING_MAP.put(2.779649, 0.331436)
    SHOOTING_MAP.put(3.291857, 0.384108)
    SHOOTING_MAP.put(3.998236, 0.436227)
    SHOOTING_MAP.put(4.935377, 0.514283)

    TIME_MAP.put(1.32, 0.20)
    TIME_MAP.put(2.29, 0.28)
    TIME_MAP.put(3.0, 0.375)
    TIME_MAP.put(3.73, 0.40)
    TIME_MAP.put(4.61, 0.50)
    TIME_MAP.put(5.24, 0.55)
  }
}
