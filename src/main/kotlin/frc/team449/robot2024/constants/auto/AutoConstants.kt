package frc.team449.robot2024.constants.auto

import kotlin.math.PI

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 1.875
  const val DEFAULT_Y_KP = 1.875
  const val DEFAULT_ROTATION_KP = 2.0

  const val ORBIT_KP = 2 * PI

  const val SHOOT_INTAKE_TIME = 0.5
  const val FEEDER_REVERSE_TIME = 0.5
}
