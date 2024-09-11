package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import kotlin.math.pow

object SpinShooterKrakenConstants {

  const val DUTY_CYCLE_DEADBAND = 0.001
  const val RIGHT_MOTOR_ID = 45
  val RIGHT_MOTOR_ORIENTATION = InvertedValue.CounterClockwise_Positive
  val RIGHT_NEUTRAL_MODE = NeutralModeValue.Coast
  const val LEFT_MOTOR_ID = 46
  val LEFT_MOTOR_ORIENTATION = InvertedValue.Clockwise_Positive
  val LEFT_NEUTRAL_MODE = NeutralModeValue.Coast

  const val UPDATE_FREQUENCY = 100.0

  const val STATOR_CURRENT_LIMIT = 150.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val BURST_CURRENT_LIMIT = 60.0
  const val BURST_TIME_LIMIT = 0.25

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

  const val LEFT_KS = 0.16948 // 0.19599
  const val RIGHT_KS = 0.25431 // 0.28982
  const val LEFT_KV = 0.061287 // 0.010993
  const val RIGHT_KV = 0.062885 // 0.010836
  const val LEFT_KA = 0.015242 // 0.0061217
  const val RIGHT_KA = 0.016153 // 0.00815
  const val LEFT_KP = 0.11403
  const val RIGHT_KP = 0.088873
  const val LEFT_KI = 0.0
  const val RIGHT_KI = 0.0
  const val LEFT_KD = 0.0
  const val RIGHT_KD = 0.0

  val IN_TOLERANCE = Units.radiansPerSecondToRotationsPerMinute(10.0) / 60
  val AIM_TOLERANCE = Units.radiansPerSecondToRotationsPerMinute(15.0) / 60
  val AMP_TOLERANCE = Units.radiansPerSecondToRotationsPerMinute(75.0) / 60
  val MIN_COAST_VEL = Units.radiansPerSecondToRotationsPerMinute(15.0) / 60

  const val GEARING = 1.0 / 2.0

  val equation = { x: Double -> -78.7 + 2.18 * x - 0.0176 * x.pow(2) + 6.82e-5 * x.pow(3) - 1e-7 * x.pow(4) }

  init {
    /**
     * Data is entered as follows:
     * SHOOTING_MAP.put(distanceToSpeaker, pivotAngle)
     */
  }
}
