package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class Experimental3PieceMid(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(),
      stopEventMap = hashMapOf(
        0 to WaitCommand(1.5),
        1 to WaitCommand(1.5),
        2 to WaitCommand(1.5)
      ),
      debug = true
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("3_Piece_Mid")
      )
    } else {
      ChoreoTrajectory.createTrajectory("3_Piece_Mid")
    }
}
