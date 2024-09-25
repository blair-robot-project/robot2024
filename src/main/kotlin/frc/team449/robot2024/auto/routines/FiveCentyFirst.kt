package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.vision.VisionConstants

class FiveCentyFirst(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {
  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoIntake(robot),
        1 to AutoUtil.autoIntake(robot),
        2 to AutoUtil.autoIntake(robot),
        3 to AutoUtil.autoIntake(robot),
        4 to AutoUtil.autoIntake(robot)
      ),
      stopEventMap = hashMapOf(
        0 to InstantCommand({
          robot.drive.enableVisionFusion = true
          VisionConstants.MAX_DISTANCE_SINGLE_TAG = 1e-6
        }).andThen(
          AutoUtil.autoShoot(robot)
        ),
        1 to AutoUtil.autoShoot(robot),
        2 to AutoUtil.autoShoot(robot),
        3 to AutoUtil.autoShoot(robot),
        4 to AutoUtil.autoShoot(robot)
          .andThen(
            InstantCommand({
              robot.drive.stop()
              VisionConstants.MAX_DISTANCE_SINGLE_TAG = 5.0
            }, robot.drive),
            robot.undertaker.stop(),
            robot.feeder.stop(),
            robot.shooter.forceStop(),
            robot.pivot.moveStow(),
          )
      ),
      debug = false,
      timeout = 0.40
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("5_Sub_Centy_First_Vision")
      )
    } else {
      ChoreoTrajectory.createTrajectory("5_Sub_Centy_First_Vision")
    }
}
