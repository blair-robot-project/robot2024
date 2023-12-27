package frc.team449.robot2023.commands.driveAlign

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.DriveSubsystem
import frc.team449.control.OI
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.auto.AutoConstants

/**
 * @param drive The holonomic drive you want to align with
 * @param point The point in 2d space you want the drivetrain to face towards
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 */
class OrbitAlign(
  private val drive: DriveSubsystem,
  private val oi: OI,
  private val point: Translation2d,
  private val headingPID: PIDController = PIDController(
    AutoConstants.ORBIT_KP,
    0.0,
    0.0
  )
) : Command() {

  init {
    addRequirements(drive)
    headingPID.enableContinuousInput(-Math.PI, Math.PI)
    headingPID.setTolerance(0.015)
  }

  private var fieldToRobot = Translation2d()
  private var robotToPoint = Translation2d()

  override fun execute() {
    fieldToRobot = drive.pose.translation
    robotToPoint = point - fieldToRobot
    headingPID.setpoint = robotToPoint.angle.radians

    drive.set(
      ChassisSpeeds(
        oi.get().vxMetersPerSecond,
        oi.get().vyMetersPerSecond,
        MathUtil.clamp(headingPID.calculate(drive.heading.radians), -RobotConstants.MAX_ROT_SPEED, RobotConstants.MAX_ROT_SPEED)
      )
    )
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
