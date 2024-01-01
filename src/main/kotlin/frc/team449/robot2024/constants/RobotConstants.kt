package frc.team449.robot2024.constants

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import frc.team449.robot2024.constants.drives.SwerveConstants
import kotlin.math.PI

object RobotConstants {

  /** Other CAN ID */
  const val PDH_CAN = 1

  /** Controller Configurations */
  const val ROT_RATE_LIMIT = 4.0 * PI
  const val NEG_ROT_RATE_LIM = -8.0 * PI
  const val TRANSLATION_DEADBAND = .15
  const val ROTATION_DEADBAND = .15

  /** In kilograms, include bumpers and battery and all */
  const val ROBOT_WEIGHT = 55

  /** Drive configuration */
  const val MAX_LINEAR_SPEED = SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED // m/s
  const val MAX_ROT_SPEED = PI // rad/s
  val MAX_ACCEL = 4 * DCMotor(
    MotorConstants.NOMINAL_VOLTAGE,
    MotorConstants.STALL_TORQUE * SwerveConstants.EFFICIENCY,
    MotorConstants.STALL_CURRENT,
    MotorConstants.FREE_CURRENT,
    MotorConstants.FREE_SPEED,
    1
  ).getTorque(55.0) /
    ((SwerveConstants.DRIVE_UPR / (2 * PI)) * ROBOT_WEIGHT * SwerveConstants.DRIVE_GEARING) // m/s/s

  val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))


  init {
    println("Max Accel $MAX_ACCEL")
  }

  const val LOOP_TIME = 0.020

  /** PID controller for Orthogonal turning */
  val ORTHOGONAL_CONTROLLER = PIDController(
    3.0,
    0.0,
    0.0
  )

  const val ALIGN_ROT_SPEED = 3 * PI / 2

  val IR_CHANNEL = 15

  // Robot dimensions (INCLUDING BUMPERS)
  val ROBOT_WIDTH = Units.inchesToMeters(27.0 + 3.25 * 2)
  val ROBOT_LENGTH = Units.inchesToMeters(30.0 + 3.25 * 2)
}
