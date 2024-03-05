package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import frc.team449.robot2024.constants.MotorConstants
import kotlin.math.PI

object PivotConstants {

  const val MOTOR_ID = 11
  const val INVERTED = false
  const val CURRENT_LIM = 40
  const val FOLLOWER_ID = 12
  const val FOLLOWER_INVERTED = false

  /** Encoder stuff */
  const val ENC_CHANNEL = 0
  const val GEARING = 1.0 / 75.0
  const val UPR = 2 * PI / 1.5
  const val OFFSET = -0.2125 + (0.150882 / UPR) + (0.829237 / UPR) - (0.012295 / UPR) +
    (0.020301 / UPR) + (0.339 / UPR)
  const val ENC_INVERTED = true

  val QUAD_ENCODER = Encoder(1, 2)
  val CPR = 2048
  val SAMPLES_TO_AVERAGE = 127

  const val NUM_MOTORS = 2

  /** Moment of inertia in m^2 kg given from CAD with a 0.035 m^2kg cushion for unmodeled objects */
  const val MOMENT_OF_INERTIA = 0.570869 + 0.035

  const val EFFICIENCY = 0.95

  const val ARM_LENGTH = 1.0

  const val KS = 0.0

  /** Deviations for Kalman filter in units of radians or radians / seconds */
  val MODEL_POS_DEVIATION = Units.degreesToRadians(10.0)
  val MODEL_VEL_DEVIATION = Units.degreesToRadians(20.0)
  const val MODEL_ERROR_DEVIATION = 0.10
  val ENCODER_POS_DEVIATION = Units.degreesToRadians(0.25)

  /** LQR Position and Velocity tolerances */
  val POS_TOLERANCE = Units.degreesToRadians(3.0)
  val VEL_TOLERANCE = Units.degreesToRadians(10.0)
  const val CONTROL_EFFORT_VOLTS = 12.0

  val MAX_VEL_TOL = Units.degreesToRadians(30.0)

  const val MAX_VOLTAGE = 12.0

  /** Profile Constraints */
  val MAX_VELOCITY = MotorConstants.FREE_SPEED * GEARING

  // Max at 40A should be 10.567679154992222
  const val MAX_ACCEL = 8.0
  const val SLOW_ACCEL = 3.0

  val MIN_ANGLE = Units.degreesToRadians(0.0)
  val MAX_ANGLE = Units.degreesToRadians(105.0)
  val AMP_ANGLE = Units.degreesToRadians(95.0)
  val CLIMB_ANGLE = Units.degreesToRadians(70.0)
  val STOW_ANGLE = Units.degreesToRadians(-2.0)

  // IS THIS CORRECT???
  val AUTO_ANGLE = 0.350
}
