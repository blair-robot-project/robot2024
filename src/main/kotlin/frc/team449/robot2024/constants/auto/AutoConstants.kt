package frc.team449.robot2024.constants.auto

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 2.55
  const val DEFAULT_Y_KP = 2.55
  const val DEFAULT_X_KD = 0.05
  const val DEFAULT_Y_KD = 0.05
  const val DEFAULT_ROTATION_KP = 2.45 // 2.45
  const val DEFAULT_ROTATION_KD = 0.05

  const val ORBIT_KP = 5.0
  const val ORBIT_KD = 0.00

  const val MAX_ROT_VEL = 10.0 // rad/s
  const val MAX_ROT_ACCEL = 10.0 // rad/s

  const val SHOOT_INTAKE_TIME = 0.35

  const val AUTO_SHOOT_TIMEOUT_SECONDS = 2.0
  const val AUTO_SPINUP_TIMEOUT_SECONDS = 2.0

  const val SHOOT_AWAY_WAIT = 0.25
}
