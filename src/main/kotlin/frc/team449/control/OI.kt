package frc.team449.control

import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * An Operator Input (OI) that gets the desired [ChassisSpeeds] to give a [DriveSubsystem].
 */

fun interface OI {
  fun get(): ChassisSpeeds
}
