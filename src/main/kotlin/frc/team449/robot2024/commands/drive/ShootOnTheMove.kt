package frc.team449.robot2024.commands.drive

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.robot2024.Robot

class ShootOnTheMove(
  private val robot: Robot,
  private val robotPose: Pose2d

) : Command() {

  private val shootTimer = Timer()

  override fun initialize() {
    shootTimer.restart()
    PrintCommand("Starting.. SHOOT ON THE MOVE").schedule()
  }

//  override fun execute() {
//    val robotVel = robot.drive.currentSpeeds
//    val robotAccel = robot.drive.fieldRelativeAccel
//
//    val robotLocation = robot.drive.pose.translation
//    val distanceToTarget = FieldConstants.SPEAKER_POSE.getDistance(robotLocation)
//    var shotTime = distanceToTravelTime.get(distanceToTarget)
//    var movingGoalLocation = Translation2d()
//
//    var i = 0
//
//    while (i < 5) {
//      val virtualGoalX: Double = FieldConstants.SPEAKER_POSE.x - shotTime * (robotVel.vxMetersPerSecond + robotAccel.ax * 0.01)
//      val virtualgoalY: Double = FieldConstants.SPEAKER_POSE.y - shotTime * (robotVel.vyMetersPerSecond + robotAccel.ay * 0.01)
//
//      val testGoalLocation = Translation2d(virtualGoalX, virtualgoalY)
//      var newShotTime = distanceToTravelTime.get(distanceToTarget)
//
//      if (abs(newShotTime - shotTime) <= 0.01) {
//        i = 4
//      }
//
//      if (i == 4) {
//        movingGoalLocation = testGoalLocation
//      } else {
//        shotTime = newShotTime
//      }
//
//      i++
//    }
//
//    robot.pivot.moveToAngle(ShooterConstants.SHOOTING_MAP.get(distanceToTarget)[0, 2])
//
//    // TODO: finish command
//  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
  }
}

class ChassisAccels(
  lastSpeeds: ChassisSpeeds,
  currentSpeeds: ChassisSpeeds,
  dt: Double
) {
  var ax = (currentSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / dt
  var ay = (currentSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / dt
  var alpha = (currentSpeeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond) / dt
}
