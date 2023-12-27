package frc.team449.control

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command

/**
 * Generic driving command that applies the OI output to the drivetrain.
 * @param drive The drivetrain to be controlled.
 * @param oi The OI that feeds the inputted [ChassisSpeeds] to the [drive].
 */
class DriveCommand(
  private val drive: DriveSubsystem,
  private val oi: OI
) : Command() {

  init {
    addRequirements(drive)
  }

  /** Take returned [ChassisSpeeds] from a joystick/[OI] and feed it to a [DriveSubsystem]. */
  override fun execute() {
    drive.set(oi.get())
  }
}
