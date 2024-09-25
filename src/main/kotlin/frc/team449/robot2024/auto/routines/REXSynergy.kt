package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class REXSynergy(
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
        3 to ParallelCommandGroup(
          robot.undertaker.intake(),
          robot.feeder.intake(),
          robot.shooter.shootSubwoofer()
        )
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoShoot(robot),
        2 to AutoUtil.autoShoot(robot),
        3 to AutoUtil.autoShoot(robot),
        4 to ParallelCommandGroup(
          robot.undertaker.intake(),
          robot.feeder.intake(),
          robot.shooter.shootSubwoofer()
        )
          .until { robot.infrared.get() && robot.closeToShooterInfrared.get() }
          .andThen(
            InstantCommand({ robot.drive.stop() }),
            robot.undertaker.stop(),
            robot.feeder.stop(),
            robot.shooter.forceStop()
          )
      ),
      debug = false,
      timeout = 0.0
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("REXSynergy")
      )
    } else {
      ChoreoTrajectory.createTrajectory("REXSynergy")
    }
}
