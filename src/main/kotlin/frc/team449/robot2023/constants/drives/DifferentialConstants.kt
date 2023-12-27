package frc.team449.robot2023.constants.drives

import edu.wpi.first.math.controller.DifferentialDriveFeedforward
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import java.lang.Math.PI

/** Constants for a differential drivetrain */
object DifferentialConstants {

  /** Drive motor CAN ID */
  const val DRIVE_MOTOR_L = 2
  const val DRIVE_MOTOR_L1 = 4
  const val DRIVE_MOTOR_L2 = 3
  const val DRIVE_MOTOR_R = 1
  const val DRIVE_MOTOR_R1 = 11
  const val DRIVE_MOTOR_R2 = 7

  /** Angular feed forward gains. */
  const val DRIVE_ANGLE_FF_KS = 0.20112
  const val DRIVE_ANGLE_FF_KV = 10.05
  const val DRIVE_ANGLE_FF_KA = 0.505

  /** Control Constants */
  const val DRIVE_KP = .0
  const val DRIVE_KI = .0
  const val DRIVE_KD = .0
  const val DRIVE_FF_KS = 0.1908
  const val DRIVE_FF_KV = 2.5406
  const val DRIVE_FF_KA = 0.44982

  /** Encoder Characteristics */
  val DRIVE_ENC_RIGHT = Encoder(4, 5)
  val DRIVE_ENC_LEFT = Encoder(6, 7)
  const val NEO_ENCODER_CPR = 1
  const val DRIVE_EXT_ENC_CPR = 256

  /** Drive Characteristics */
  const val DRIVE_GEARING = 1.0
  val DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2.0)
  val DRIVE_UPR = 2 * PI * DRIVE_WHEEL_RADIUS
  const val DRIVE_CURRENT_LIM = 35
  const val DRIVE_ENC_VEL_THRESHOLD = 999999.0
  const val TRACK_WIDTH = .615 // m

  val DRIVE_FEED_FORWARD = DifferentialDriveFeedforward(
    DRIVE_FF_KV,
    DRIVE_FF_KA,
    DRIVE_ANGLE_FF_KV,
    DRIVE_ANGLE_FF_KA,
    TRACK_WIDTH
  )
}
