package frc.team449.control

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Subsystem

/** A drivetrain that uses closed-loop velocity control. */
interface DriveSubsystem : Subsystem {
  var heading: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(this.pose.rotation.radians))
    set(value) {
      this.pose = Pose2d(Translation2d(this.pose.x, this.pose.y), value)
    }

  var pose: Pose2d

  /** Sets the drivetrain's desired speeds. */
  fun set(desiredSpeeds: ChassisSpeeds)

  /** Sets all the robot's drive motors to 0. */
  fun stop()

  /**
   * Used to simulate a drivetrain. Only one instance of this class should be made per drivetrain.
   */
  interface SimController {
    fun update()

    /**
     * Simulate the current drawn by the drivetrain
     */
    fun getCurrentDraw(): Double
  }
}
