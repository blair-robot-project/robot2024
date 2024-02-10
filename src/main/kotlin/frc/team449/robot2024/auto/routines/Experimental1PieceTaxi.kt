package frc.team449.robot2024.auto.routines

import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class Experimental1PieceTaxi(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot)
      ),
      debug = true
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("1_Piece_Taxi")
      )
    } else {
      ChoreoTrajectory.createTrajectory("1_Piece_Taxi")
    }
}
