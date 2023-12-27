package frc.team449.robot2023.constants

import edu.wpi.first.math.util.Units

object MotorConstants {
  /** NEO characteristics */
  const val NOMINAL_VOLTAGE = 12.0
  const val STALL_TORQUE = 3.36
  const val STALL_CURRENT = 166.0
  const val FREE_CURRENT = 1.3
  val FREE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5676.0)
}
