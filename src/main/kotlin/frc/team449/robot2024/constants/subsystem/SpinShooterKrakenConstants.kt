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
  val LEFT_MOTOR_ORIENTATION = InvertedValue.CounterClockwise_Positive
  val LEFT_NEUTRAL_MODE = NeutralModeValue.Coast

  const val UPDATE_FREQUENCY = 100.0

  const val STATOR_CURRENT_LIMIT = 150.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val BURST_CURRENT_LIMIT = 60.0
  const val BURST_TIME_LIMIT = 0.25

  const val SUBWOOFER_LEFT_SPEED = 3450.0 / 60
  const val SUBWOOFER_RIGHT_SPEED = 2800.0 / 60
  const val ANYWHERE_LEFT_SPEED = 4950.0 / 60
  const val ANYWHERE_RIGHT_SPEED = 4250.0 / 60
  const val PASS_LEFT_SPEED = 3900.0 / 60
  const val PASS_RIGHT_SPEED = 2900.0 / 60
  const val PASS2_LEFT_SPEED = 3550.0 / 60
  const val PASS2_RIGHT_SPEED = 2550.0 / 60
  const val PASS3_LEFT_SPEED = 3550.0 / 60
  const val PASS3_RIGHT_SPEED = 2550.0 / 60
  const val AMP_SPEED = 1800.0 / 60
  const val OUTTAKE_SPEED = -200.0 / 60

  val MIN_COAST_VEL = Units.radiansToRotations(15.0)
  val AUTO_SHOOT_TOL = Units.radiansToRotations(25.0)
  const val BRAKE_RATE_LIMIT = 5250.0 / 60

  val SHOOTING_MAP = InterpolatingDoubleTreeMap()
  val TIME_MAP = InterpolatingDoubleTreeMap()

  val IN_TOLERANCE = Units.radiansToRotations(10.0)
  val AIM_TOLERANCE = Units.radiansToRotations(15.0)
  val AMP_TOLERANCE = Units.radiansToRotations(75.0)

  const val GEARING = 1.0 / 2.0

  const val LEFT_KS = 0.0 // 0.2377
  const val RIGHT_KS = 0.0 // 0.32957
  const val LEFT_KV = 12.0 * GEARING / 100 // nominal motor voltage / max motor speed [in rotations per second]
  const val RIGHT_KV = 12.0 * GEARING / 100 // nominal motor voltage / max motor speed [in rotations per second]
  const val LEFT_KP = 1.0
  const val RIGHT_KP = 1.0
  const val LEFT_KI = 0.0
  const val RIGHT_KI = 0.0
  const val LEFT_KD = 0.0
  const val RIGHT_KD = 0.0

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
