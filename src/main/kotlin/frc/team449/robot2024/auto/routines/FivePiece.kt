package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class FivePiece(
  robot: Robot,
  alliance: Alliance
) : ChoreoRoutineStructure {
  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoIntakeAway(robot),
        1 to AutoUtil.autoIntakeAway(robot),
        2 to AutoUtil.autoIntakeAway(robot),
        3 to AutoUtil.autoIntakeAway(robot),
        4 to AutoUtil.autoIntakeAway(robot)
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoShootAway(robot),
        2 to AutoUtil.autoShootAway(robot),
        3 to AutoUtil.autoShootAway(robot),
        4 to AutoUtil.autoShootAway(robot)
      ),
      debug = false
    )

  override val trajectory: MutableList<ChoreoTrajectory> = ChoreoTrajectory.createTrajectory(
    "4_Piece_Shoot_Anywhere",
    alliance
  )
}
