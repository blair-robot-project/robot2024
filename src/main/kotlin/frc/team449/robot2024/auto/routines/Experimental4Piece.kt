package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class Experimental4Piece(
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
        ChoreoTrajectory.createTrajectory("Experimental-4_Piece")
      )
    } else {
      ChoreoTrajectory.createTrajectory("Experimental-4_Piece")
    }
}
