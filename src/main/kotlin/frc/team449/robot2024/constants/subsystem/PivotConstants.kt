package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotBase
import frc.team449.robot2024.constants.MotorConstants
import kotlin.math.PI

object PivotConstants {

  const val MOTOR_ID = 11
  const val INVERTED = false
  const val CURRENT_LIM = 50
  const val FOLLOWER_ID = 12
  const val FOLLOWER_INVERTED = false

  /** Encoder stuff */
  const val ENC_CHANNEL = 0
  const val GEARING = 1.0 / 75.0
  const val UPR = 2.0 * PI * (26.0 / 36.0)
  const val OFFSET = - (0.3333333134651184 / UPR)
  const val ENC_INVERTED = true

  val QUAD_ENCODER = Encoder(1, 2)
  val CPR = 2048
  val SAMPLES_TO_AVERAGE = 127

  const val NUM_MOTORS = 2

  /** Moment of inertia in m^2 kg given from CAD with a 0.085 m^2kg cushion for unmodeled objects */
  const val MOMENT_OF_INERTIA = 0.57125221 + 0.085

  const val EFFICIENCY = 0.875

  const val ARM_LENGTH = 0.4027778

  val SIMPLE_FF = ArmFeedforward(
    if (RobotBase.isReal()) 0.13496 else 0.0,
    if (RobotBase.isReal()) 0.34977 else 0.0,
    0.0
  )

  val KG_OFFSET = Units.degreesToRadians(8.343)

  /** Deviations for Kalman filter in units of radians or radians / seconds */
  val MODEL_POS_DEVIATION = Units.degreesToRadians(10.0)
  val MODEL_VEL_DEVIATION = Units.degreesToRadians(20.0)
  const val MODEL_INPUT_ERROR_DEVIATION = 0.050
  val ENCODER_POS_DEVIATION = Units.degreesToRadians(0.025)

  val START_INPT_ERR = Units.degreesToRadians(5.0)

  /** LQR Position and Velocity tolerances */
  val POS_TOLERANCE = Units.degreesToRadians(1.975)
  val VEL_TOLERANCE = Units.degreesToRadians(13.5)
  const val CONTROL_EFFORT_VOLTS = 12.0

  /** LQR Position and Velocity tolerances for fast Controller*/
  // for offseason, increase these tolerances to like 20.0 and 60.0 or something
  val FAST_POS_TOLERANCE = Units.degreesToRadians(10.0)
  val FAST_VEL_TOLERANCE = Units.degreesToRadians(45.0)

  val AUTO_POS_TOLERANCE = Units.degreesToRadians(1.0)
  val AUTO_VEL_TOLERANCE = Units.degreesToRadians(10.0)
  val AUTO_MAX_POS_ERROR = Units.degreesToRadians(0.90)

  val SHOOT_ANYWHERE_POS_TOLERANCE = Units.degreesToRadians(0.95)

  val MAX_POS_ERROR = Units.degreesToRadians(1.65)
  val AMP_TOL_RANGE = 75.0..115.0
  val AMP_VEL_TOL = Units.degreesToRadians(115.0)
  val STOW_TOL = Units.degreesToRadians(12.5)
  val MAX_VEL_ERROR = Units.degreesToRadians(30.0)

  const val MAX_VOLTAGE = 12.0

  /** Profile Constraints */
  val MAX_VELOCITY = MotorConstants.FREE_SPEED * GEARING * 0.85

  // Max at 35A should be 178.15455320820428 (at stall)
  // for offseason, try increasing this to 175 :joy:
  val MAX_ACCEL = 0.1727 * 195 // this some fire, actual value is 33.6765

  const val SLOW_ACCEL = 0.5804 * 21.75 // this some fire, actual value is 12.6237

  const val AUTO_ACCEL = 2.75

  val MIN_ANGLE = Units.degreesToRadians(0.0)
  val MAX_ANGLE = Units.degreesToRadians(105.0)
  val AMP_ANGLE = Units.degreesToRadians(89.0) // 95.0 at blacksburg
  val CLIMB_ANGLE = Units.degreesToRadians(60.0)
  val PASS_ANGLE = Units.degreesToRadians(7.5)
  val PASS_ANGLE_T2 = Units.degreesToRadians(5.0)
  val PASS_ANGLE_T3 = Units.degreesToRadians(0.0)
  val STOW_ANGLE = Units.degreesToRadians(-1.0)
}
