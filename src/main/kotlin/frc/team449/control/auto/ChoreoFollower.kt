package frc.team449.control.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2024.constants.auto.AutoConstants
import kotlin.math.PI

/**
 * Follower command so a robot can follow a parsed Choreo Trajectory with feedback
 * @see ChoreoTrajectory
 * @param drivetrain Swerve Drivetrain to use
 * @param trajectory Parsed Choreo Trajectory
 * @param xController PID Controller to use for x-position error (output is next desired x velocity, not volts)
 * @param yController PID Controller to use for y-position error (output is next desired y velocity, not volts)
 * @param thetaController PID Controller to use for rotation error (output is next desired rotation velocity, not volts)
 * @param poseTol Tolerance within final pose to say it is "good enough"
 * @param timeout Maximum time to wait after trajectory has finished to get in tolerance. A very low timeout may end this command before you get in tolerance.
 * @param resetPose Whether to reset the pose to the first pose in the trajectory
 * @param debug Whether to run on trajectory expected velocities only (no feedback control)
 */
class ChoreoFollower(
  private val drivetrain: SwerveDrive,
  private val trajectory: ChoreoTrajectory,
  private val xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  private val yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  private val thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  poseTol: Pose2d = Pose2d(0.035, 0.035, Rotation2d(0.035)),
  private val timeout: Double = 0.65,
  private val resetPose: Boolean = false,
  private val debug: Boolean = false
) : Command() {

  private val timer = Timer()

  init {
    addRequirements(drivetrain)

    xController.reset()
    xController.setTolerance(poseTol.x)

    yController.reset()
    yController.setTolerance(poseTol.y)

    thetaController.reset()
    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(poseTol.rotation.radians)
  }

  private fun calculate(currPose: Pose2d, desState: ChoreoTrajectory.ChoreoState): ChassisSpeeds {
    val xFF = desState.xVel
    val yFF = desState.yVel
    val angFF = desState.thetaVel

    val xPID = xController.calculate(currPose.x, desState.xPos)
    val yPID = yController.calculate(currPose.y, desState.yPos)
    val angPID = thetaController.calculate(currPose.rotation.radians, desState.theta)

    return if (debug) {
      ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, angFF, currPose.rotation)
    } else {
      ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xPID, yFF + yPID, angFF + angPID, currPose.rotation)
    }
  }

  private fun allControllersAtReference(): Boolean {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()
  }

  override fun initialize() {
    xController.reset()
    yController.reset()
    thetaController.reset()

    if (resetPose) {
      drivetrain.pose = trajectory.initialPose()
    }

    timer.restart()
  }

  override fun execute() {
    val currTime = timer.get()

    val desiredMatrix = trajectory.sample(currTime)

    drivetrain.set(calculate(drivetrain.pose, desiredMatrix))
  }

  override fun isFinished(): Boolean {
    return (timer.hasElapsed(trajectory.totalTime) && allControllersAtReference()) ||
      timer.hasElapsed(trajectory.totalTime + timeout)
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    drivetrain.stop()
  }
}
