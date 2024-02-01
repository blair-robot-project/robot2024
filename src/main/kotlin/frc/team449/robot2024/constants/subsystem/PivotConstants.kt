package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.util.Units
import kotlin.math.PI

object PivotConstants {

  const val MOTOR_ID = 46
  const val INVERTED = false
  const val CURRENT_LIM = 40
  const val GEARING = 1.0 / 75.0
  const val UPR = 2 * PI
  const val FOLLOWER_ID = 47
  const val FOLLOWER_INVERTED = true

  const val NUM_MOTORS = 2
  const val MOMENT_OF_INERTIA = 2.0

  const val EFFICIENCY = 1.0

  const val ARM_LENGTH = 1.0

  /** Deviations for Kalman filter in units of radians or radians / seconds */
  val MODEL_POS_DEVIATION = Units.degreesToRadians(10.0)
  val MODEL_VEL_DEVIATION = Units.degreesToRadians(20.0)
  val ENCODER_POS_DEVIATION = Units.degreesToRadians(0.25)

  /** LQR Position and Velocity tolerances */
  val POS_TOLERANCE = Units.degreesToRadians(0.25)
  val VEL_TOLERANCE = Units.degreesToRadians(10.0)
  val CONTROL_EFFORT_VOLTS = 12.0

  val MAX_VOLTAGE = 12.0

  /** Profile Constraints */
  val MAX_VELOCITY = Units.degreesToRadians(100.0)
  val MAX_ACCEL = Units.degreesToRadians(75.0)

  val MIN_ANGLE = Units.degreesToRadians(0.0)
  val MAX_ANGLE = Units.degreesToRadians(90.0)
  val AMP_ANGLE = Units.degreesToRadians(75.0)
  val SUBWOOFER_ANGLE = Units.degreesToRadians(30.0)
}
