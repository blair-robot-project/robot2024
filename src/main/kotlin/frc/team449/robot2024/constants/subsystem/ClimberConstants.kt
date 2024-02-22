package frc.team449.robot2024.constants.subsystem

object ClimberConstants {
  const val RIGHT_ID = 13
  const val LEFT_ID = 10

  const val RIGHT_INVERTED = false
  const val LEFT_INVERTED = false

  const val CURRENT_LIM = 40

  const val RETRACT_VOLTAGE = -12.0
  const val EXTEND_VOLTAGE = 3.0

  /** What is the max enc pos? */
  const val MIN_ENC_POS = 0.0
  const val MAX_ENC_POS = Double.MAX_VALUE

  const val DEFAULT_PID_RETRACT = -7.0
  const val KP = 1.0
  const val KI = 0.0
  const val KD = 0.0

  const val MAX_SIM_POS = 1.5
  const val SIM_SPEED = 0.25
}
