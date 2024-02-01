package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3

object ShooterConstants {
  const val EFFICIENCY = 1.0
  const val RIGHT_MOTOR_ID = 60
  const val RIGHT_MOTOR_INVERTED = false
  const val LEFT_MOTOR_ID = 61
  const val LEFT_MOTOR_INVERTED = false
  const val CURRENT_LIMIT = 40

  const val SUBWOOFER_LEFT_SPEED = 100.0
  const val SUBWOOFER_RIGHT_SPEED = 125.0

  val SHOOTING_MAP = InterpolatingMatrixTreeMap<Double, N3, N1>()

  const val LEFT_KS = 0.0
  const val RIGHT_KS = 0.0

  const val AMP_SCORE_VOLTAGE = 4.0
  const val DURING_INTAKE_VOLTAGE = -3.0

  /** In meters from the ground */
  const val SHOOTER_HEIGHT = 0.25

  /** These constants are PER SIDE of the shooter */
  const val MOMENT_OF_INERTIA = 2.50
  const val NUM_MOTORS = 1

  const val MODEL_VEL_STDDEV = 3.0
  const val ENCODER_VEL_STDDEV = 0.01
  const val LQR_VEL_TOL = 10.0 / 60.0
  const val LQR_MAX_VOLTS = 12.0
  const val MAX_VOLTAGE = 12.0

  /** Encoder stuff */
  const val LEFT_CHANNEL_A = 1
  const val LEFT_CHANNEL_B = 2
  const val RIGHT_CHANNEL_A = 3
  const val RIGHT_CHANNEL_B = 4
  const val CPR = 2048
  const val UPR = 1.0
  const val GEARING = 1.0 / 1.0
  const val LEFT_ENCODER_INVERTED = false
  const val RIGHT_ENCODER_INVERTED = false

  init {
    /**
     * Fill with values of optimized left/right and pivot angles
     *  for a given distance to the Speaker
     *
     * It may be better to mathematically calculate pivot angle,
     *  this is something to test
     *
     * Data is entered as following:
     *  Right shooter speed, left shooter speed, pivot angle
     */
    SHOOTING_MAP.put(
      0.0,
      MatBuilder.fill(
        Nat.N3(),
        Nat.N1(),
        0.0,
        0.0,
        0.0
      )
    )
  }
}
