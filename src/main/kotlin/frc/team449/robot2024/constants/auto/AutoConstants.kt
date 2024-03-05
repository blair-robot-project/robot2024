package frc.team449.robot2024.constants.auto

import edu.wpi.first.math.controller.PIDController

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 2.55
  const val DEFAULT_Y_KP = 2.55

  const val DEFAULT_X_KI = 0.0
  const val DEFAULT_Y_KI = 0.0

  const val DEFAULT_X_KD = 0.05
  const val DEFAULT_Y_KD = 0.05

  const val DEFAULT_ROTATION_KP = 2.45
  const val DEFAULT_ROTATION_KI = 0.0
  const val DEFAULT_ROTATION_KD = 0.05

  val X_CONTROLLER = PIDController(DEFAULT_X_KP, DEFAULT_X_KI, DEFAULT_X_KD)
  val Y_CONTROLLER = PIDController(DEFAULT_Y_KP, DEFAULT_Y_KI, DEFAULT_Y_KD)
  val ROT_CONTROLLER = PIDController(DEFAULT_ROTATION_KP, DEFAULT_ROTATION_KI, DEFAULT_ROTATION_KD)

  const val ORBIT_KP = 2.0

  const val SHOOT_INTAKE_TIME = 0.35

  const val AUTO_SHOOT_TIMEOUT_SECONDS = 2.0
  const val AUTO_SPINUP_TIMEOUT_SECONDS = 2.0

  const val SHOOT_AWAY_WAIT = 0.25
}
