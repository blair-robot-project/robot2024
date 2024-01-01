package frc.team449.robot2024.commands.driveAlign

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.auto.AutoConstants
import kotlin.math.PI
import kotlin.math.hypot

/**
 * @param drive The holonomic drive you want to align with
 * @param targetPose The pose you want to drive up to
 * @param xPID The profiled PID controller with constraints you want to use for fixing X error
 * @param yPID The profiled PID controller with constraints you want to use for fixing Y error
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 * @param tolerance The allowed tolerance from the targetPose
 */
class ProfiledPoseAlign(
  private val drive: SwerveDrive,
  private val targetPose: Pose2d,
  private val xSpeed: Double,
  private val ySpeed: Double,
  private val xPID: PIDController = PIDController(
    AutoConstants.DEFAULT_X_KP,
    0.0,
    0.0
  ),
  private val yPID: PIDController = PIDController(
    AutoConstants.DEFAULT_Y_KP,
    0.0,
    0.0
  ),
  private val headingPID: PIDController = PIDController(
    AutoConstants.DEFAULT_ROTATION_KP,
    0.0,
    0.0
  ),
  private val xProfile: TrapezoidProfile = TrapezoidProfile(TrapezoidProfile.Constraints(RobotConstants.MAX_LINEAR_SPEED - 1.25, 2.25)),
  private val yProfile: TrapezoidProfile = TrapezoidProfile(TrapezoidProfile.Constraints(RobotConstants.MAX_LINEAR_SPEED - 1.25, 2.25)),
  private val tolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05)),
  private val speedTol: Double = 0.05,
  private val speedTolRot: Double = 0.05
) : Command() {
  init {
    addRequirements(drive)
  }

  val timer = Timer()

  override fun initialize() {
    headingPID.enableContinuousInput(-PI, PI)

    // Set tolerances from the given pose tolerance
    xPID.setTolerance(tolerance.x)
    yPID.setTolerance(tolerance.y)
    headingPID.setTolerance(tolerance.rotation.radians)

    headingPID.setpoint = targetPose.rotation.radians

    timer.restart()
  }

  fun getTime(): Double {
    return maxOf(xProfile.totalTime(), yProfile.totalTime(), 0.5)
  }

  override fun execute() {
    val currTime = timer.get()

    // Calculate the feedback for X, Y, and theta using their respective controllers

    val xProfCalc = xProfile.calculate(
      currTime,
      TrapezoidProfile.State(targetPose.x, 0.0),
      TrapezoidProfile.State(drive.pose.x, xSpeed)
    )

    val yProfCalc = yProfile.calculate(
      currTime,
      TrapezoidProfile.State(targetPose.y, 0.0),
      TrapezoidProfile.State(drive.pose.y, ySpeed)
    )

    val xFeedback = xPID.calculate(drive.pose.x, xProfCalc.position)
    val yFeedback = yPID.calculate(drive.pose.y, yProfCalc.position)
    val headingFeedback = headingPID.calculate(drive.heading.radians)

    drive.set(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedback + xProfCalc.velocity,
        yFeedback + yProfCalc.velocity,
        headingFeedback,
        drive.heading
      )
    )
  }

  override fun isFinished(): Boolean {
    val currTime = timer.get()

    return xPID.atSetpoint() && yPID.atSetpoint() && headingPID.atSetpoint() &&
      xProfile.isFinished(currTime) && yProfile.isFinished(currTime) &&
      hypot(
        drive.currentSpeeds.vxMetersPerSecond,
        drive.currentSpeeds.vyMetersPerSecond
      ) < speedTol &&
      drive.currentSpeeds.omegaRadiansPerSecond < speedTolRot
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
