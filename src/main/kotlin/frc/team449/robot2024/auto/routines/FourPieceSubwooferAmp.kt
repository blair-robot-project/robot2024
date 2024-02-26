package frc.team449.robot2024.auto.routines

import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class FourPieceSubwooferAmp(
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
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoShoot(robot),
        2 to AutoUtil.autoShoot(robot),
        3 to AutoUtil.autoShoot(robot).andThen(
          robot.undertaker.stop(),
          robot.feeder.stop(),
          robot.shooter.forceStop()
        )
      ),
      timeout = 0.25,
      debug = false
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("4_Piece_Subwoofer_AmpSide")
      )
    } else {
      ChoreoTrajectory.createTrajectory("4_Piece_Subwoofer_AmpSide")
    }
}
