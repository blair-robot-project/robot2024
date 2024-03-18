package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object ShooterConstants {
  const val EFFICIENCY = 0.8
  const val RIGHT_MOTOR_ID = 45
  val RIGHT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive
  const val LEFT_MOTOR_ID = 46
  val LEFT_MOTOR_OPPOSITE = true
  const val CURRENT_LIMIT = 100.0
  val BRAKE_MODE = NeutralModeValue.Coast

  val SUBWOOFER_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3478.0)
  val AUTO_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4350.0)
  val AMP_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2000.0)
  val OUTTAKE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(-200.0)

  const val AUTO_SHOOT_TOL = 30.0

  val BRAKE_RATE_LIMIT = Units.rotationsPerMinuteToRadiansPerSecond(4000.0)

  const val KS = 0.075
  const val KV = 0.012718
  const val KA = 0.010

  const val IN_TOLERANCE = 6.5

  const val NUM_MOTORS = 2

  const val MODEL_VEL_STDDEV = 3.0
  const val INPT_ERR_STDDEV = 0.000575
  const val ENCODER_VEL_STDDEV = 0.01
  const val LQR_VEL_TOL = 15.0
  const val LQR_MAX_VOLTS = 12.0
  const val MAX_VOLTAGE = 12.0
  const val ENCODER_DELAY = 0.035

  const val MIN_COAST_VEL = 15.0

  /** Encoder stuff */
  const val INTERNAL_ENC_DEPTH = 4
  const val INTERNAL_MEASUREMENT_PD = 24
  const val UPR = 2 * PI
  const val GEARING = 2.0 / 1.0
}
