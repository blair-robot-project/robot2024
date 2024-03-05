package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.controller.PIDController
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
      xController = PIDController(2.875, 0.0, 0.05),
      yController = PIDController(2.875, 0.0, 0.05),
      thetaController = PIDController(2.50, 0.0, 0.05),
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoIntake(robot),
        1 to AutoUtil.autoIntake(robot),
        2 to AutoUtil.autoIntake(robot),
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoShoot(robot),
        2 to AutoUtil.autoShoot(robot)
      ),
      debug = false
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
