package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import kotlin.math.pow

object SpinShooterKrakenConstants {
  const val RIGHT_MOTOR_ID = 20
  val RIGHT_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive
  const val LEFT_MOTOR_ID = 21
  val LEFT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive
  const val STATOR_CURRENT_LIMIT = 150.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val SUPPLY_PEAK_LIMIT = 60.0
  const val SUPPLY_PEAK_TIME = 0.25
  val NEUTRAL_MODE = NeutralModeValue.Coast

  private val RPMtoRPS: (Double) -> Double = { value: Double -> value / 60}

  val SUBWOOFER_LEFT_SPEED = RPMtoRPS(3450.0)
  val SUBWOOFER_RIGHT_SPEED = RPMtoRPS(2800.0)
  val ANYWHERE_LEFT_SPEED = RPMtoRPS(4950.0)
  val ANYWHERE_RIGHT_SPEED = RPMtoRPS(4250.0)
  val PASS_LEFT_SPEED = RPMtoRPS(3900.0)
  val PASS_RIGHT_SPEED = RPMtoRPS(2900.0)
  val PASS2_LEFT_SPEED = RPMtoRPS(3550.0)
  val PASS2_RIGHT_SPEED = RPMtoRPS(2550.0)
  val PASS3_LEFT_SPEED = RPMtoRPS(3550.0)
  val PASS3_RIGHT_SPEED = RPMtoRPS(2550.0)
  val AMP_SPEED = RPMtoRPS(1800.0)
  val OUTTAKE_SPEED = RPMtoRPS(-200.0)

  val MIN_COAST_VEL = Units.radiansToRotations(15.0)

  val AUTO_SHOOT_TOL = Units.radiansToRotations(25.0)

  val BRAKE_RATE_LIMIT = RPMtoRPS(5250.0)

  val SHOOTING_MAP = InterpolatingDoubleTreeMap()
  val TIME_MAP = InterpolatingDoubleTreeMap()

  val IN_TOLERANCE = Units.radiansToRotations(10.0)
  val AIM_TOLERANCE = Units.radiansToRotations(15.0)
  val AMP_TOLERANCE = Units.radiansToRotations(75.0)

  val kP = 0.35
  val kD = 0.05
  val kI = 0.0

  /** Encoder stuff */
  const val UPR = 1.0
  const val GEARING = 1.0 / 2.0

  const val LEFT_KS = 0.0
  const val RIGHT_KS = 0.0
  val LEFT_KV = GEARING / (Units.radiansToRotations(DCMotor.getKrakenX60(1).KvRadPerSecPerVolt))
  val RIGHT_KV = GEARING / (Units.radiansToRotations(DCMotor.getKrakenX60(1).KvRadPerSecPerVolt))

  val equation = { x: Double -> -78.7 + 2.18 * x - 0.0176 * x.pow(2) + 6.82e-5 * x.pow(3) - 1e-7 * x.pow(4) }

  init {
    /**
     * Data is entered as follows:
     * SHOOTING_MAP.put(distanceToSpeaker, pivotAngle)
     */
    print(LEFT_KV)

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
