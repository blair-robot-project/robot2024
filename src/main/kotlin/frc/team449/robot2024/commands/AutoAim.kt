package frc.team449.robot2024.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import kotlin.math.abs

class AutoAim(
  private val robot: Robot
) : Command() {

  private val shootingMap = SpinShooterConstants.SHOOTING_MAP

  override fun initialize() {
    if (getDistanceToSpeaker() > SpinShooterConstants.MAX_RANGE) {
      this.cancel()
    }

    addRequirements(robot.shooter, robot.pivot)
  }

  override fun execute() {
    robot.shooter.shootSubwoofer()
    robot.pivot.moveToAngle(shootingMap.get(getDistanceToSpeaker()))

    val fieldToRobot = robot.drive.pose.translation
    val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot
    robot.driveCommand.snapToAngle(robotToPoint.angle.radians)

    if ( // robot.mechController. &&
      abs(robot.drive.currentSpeeds.vxMetersPerSecond) <= 0.1 && abs(robot.drive.currentSpeeds.vyMetersPerSecond) <= 0.1 && abs(robot.drive.currentSpeeds.omegaRadiansPerSecond) <= 0.1
    ) {
      SequentialCommandGroup(
        WaitUntilCommand { robot.shooter.atSetpoint() },
        robot.feeder.intake(),
        robot.undertaker.intake()
      ).alongWith(
        robot.shooter.shootSubwoofer()
      ).schedule()
    }
  }

  override fun isFinished(): Boolean {
    return getDistanceToSpeaker() > SpinShooterConstants.MAX_RANGE
  }

  override fun end(interrupted: Boolean) {
    robot.undertaker.stop()
    robot.shooter.rampStop()
    robot.feeder.stop()
    robot.pivot.moveStow()
  }

  private fun getDistanceToSpeaker(): Double {
    return abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation))
  }
}
