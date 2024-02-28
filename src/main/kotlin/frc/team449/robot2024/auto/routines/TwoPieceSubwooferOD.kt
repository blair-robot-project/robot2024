package frc.team449.robot2024.auto.routines

import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class TwoPieceSubwooferOD(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {
  override val routine: ChoreoRoutine = ChoreoRoutine(
    drive = robot.drive,
    parallelEventMap = hashMapOf(),
    stopEventMap = hashMapOf(
      0 to AutoUtil.detectPiece(robot),
      1 to AutoUtil.autoShoot(robot)
    ),
    timeout = 0.25,
    debug = false
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("2_Piece_Sub_OD")
      )
    } else {
      ChoreoTrajectory.createTrajectory("2_Piece_Sub_OD")
    }
}
