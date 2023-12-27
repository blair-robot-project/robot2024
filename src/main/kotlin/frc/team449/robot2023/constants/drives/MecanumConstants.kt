package frc.team449.robot2023.constants.drives

import edu.wpi.first.math.util.Units

object MecanumConstants {

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 1
  const val DRIVE_MOTOR_FR = 2
  const val DRIVE_MOTOR_BL = 3
  const val DRIVE_MOTOR_BR = 4

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.16475
  const val DRIVE_KV = 2.0909
  const val DRIVE_KA = 0.29862

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.35
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  const val DRIVE_GEARING = 1 / 8.0
  val DRIVE_UPR = Math.PI * Units.inchesToMeters(6.0)
  const val MAX_ATTAINABLE_WHEEL_SPEED = (12 - DRIVE_KS) / DRIVE_KV
  val WHEELBASE = Units.inchesToMeters(21.426)
  val TRACKWIDTH = Units.inchesToMeters(21.000)
  const val CURRENT_LIM = 40
}
