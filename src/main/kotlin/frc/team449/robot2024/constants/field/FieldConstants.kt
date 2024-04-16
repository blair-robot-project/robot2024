package frc.team449.robot2024.constants.field

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units

object FieldConstants {
  const val fieldLength = 16.54175
  const val fieldWidth = 8.21055

  /** Find these constants in meters for the blue alliance */
  val BLUE_SPEAKER_POSE = Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42))
  val RED_SPEAKER_POSE = Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42))

  val RED_PASS_POSE = Translation2d(14.50, 6.746)
  val BLUE_PASS_POSE = Translation2d(fieldLength - 14.50, 6.746)

  var SPEAKER_POSE = BLUE_SPEAKER_POSE
  var PASS_POSE = BLUE_PASS_POSE
}
