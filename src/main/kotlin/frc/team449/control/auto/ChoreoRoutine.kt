package frc.team449.control.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2024.constants.auto.AutoConstants
import kotlin.math.abs

/**
 * Class for easily creating an autonomous routine command given Choreo Trajectories and parallel/stop events
 * @param xController PID Controller to use for x-position error (output is next desired x velocity, not volts)
 * @param yController PID Controller to use for y-position error (output is next desired y velocity, not volts)
 * @param thetaController PID Controller to use for rotation error (output is next desired rotation velocity, not volts)
 * @param drive Swerve Drivetrain to use
 * @param stopEventMap Map stop event commands in between trajectories. Index 0 is before the first trajectory, index n is after the nth trajectory.
 * @param parallelEventMap Map parallel commands to be run during trajectory following. This cannot require the same drive subsystem. Index 0 refers to the command during the first trajectory.
 * @param poseTol Tolerance within final pose to say it is "good enough"
 * @param resetPosition Whether to reset the pose to the first pose in every trajectory. By default you reset to the first pose in the first trajectory, unless you are in resetPositionTolerance.
 * @param resetPositionTolerance If within this tolerance from initial pose to the first pose in the first trajectory, do not reset pose.
 * @param timeout Maximum time to wait after trajectory has finished to get in tolerance. A very low timeout may end this command before you get in tolerance.
 * @param debug Whether to run on trajectory expected velocities only (no feedback control)
 */
class ChoreoRoutine(
  private val xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, AutoConstants.DEFAULT_X_KD),
  private val yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, AutoConstants.DEFAULT_Y_KD),
  private val thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, AutoConstants.DEFAULT_ROTATION_KD),
  private val drive: SwerveDrive,
  private val stopEventMap: HashMap<Int, Command> = HashMap(),
  private val parallelEventMap: HashMap<Int, Command> = HashMap(),
  private val poseTol: Pose2d = Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1.5)),
  private val resetPosition: Boolean = false,
  private val resetPositionTolerance: Pose2d = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
  private val timeout: Double = 0.65,
  private val debug: Boolean = false
) {

  private fun resetPose(trajectory: ChoreoTrajectory): Command {
    val poseError = drive.pose.relativeTo(trajectory.initialPose())

    if (abs(poseError.x) < resetPositionTolerance.x &&
      abs(poseError.y) < resetPositionTolerance.y &&
      abs(poseError.rotation.radians) < resetPositionTolerance.rotation.radians
    ) {
      return PrintCommand("Pose not reset.")
    }

    return InstantCommand({ drive.pose = trajectory.initialPose() })
  }

  /**
   * Create a command that follows all specified actions for an auto routine.
   * @param trajectories The list of parsed Choreo trajectories you want to run
   */
  fun createRoutine(trajectories: MutableList<ChoreoTrajectory>): Command {
    val ezraGallun = SequentialCommandGroup(
      resetPose(trajectories[0]),
      stopEventMap.getOrDefault(0, InstantCommand())
    )

    for (i in 0 until trajectories.size) {
      ezraGallun.addCommands(
        ParallelDeadlineGroup(
          ChoreoFollower(
            drive,
            trajectories[i],
            xController,
            yController,
            thetaController,
            poseTol,
            timeout,
            resetPosition,
            debug
          ),
          parallelEventMap.getOrDefault(i, InstantCommand())
        )
      )
      ezraGallun.addCommands(stopEventMap.getOrDefault(i + 1, InstantCommand()))
    }

    return ezraGallun
  }
}
