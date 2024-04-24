package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.vision.VisionConstants

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
        0 to AutoUtil.autoIntakeCenterline(robot),
        1 to AutoUtil.autoIntakeCenterline(robot),
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot).andThen(
          InstantCommand({
            robot.drive.enableVisionFusion = true
            VisionConstants.MAX_DISTANCE_SINGLE_TAG = 4.5
            VisionConstants.SINGLE_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .325, .325, 3.0))
            VisionConstants.MULTI_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .325, .325, 3.0))
          })
        ),
        1 to AutoUtil.autoShoot(robot),
        2 to AutoUtil.autoShoot(robot)
          .andThen(
            InstantCommand({
              VisionConstants.MAX_DISTANCE_SINGLE_TAG = 5.0
              VisionConstants.SINGLE_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .15, .15, 3.0))
              VisionConstants.MULTI_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .10, .10, 3.0))
            }),
            InstantCommand({ robot.drive.stop() }),
            robot.undertaker.stop(),
            robot.feeder.stop(),
            WaitCommand(0.050),
            robot.shooter.forceStop(),
            robot.pivot.moveStow(),
          )
      ),
      debug = false
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("3_Piece_Mid_ADJUSTED")
      )
    } else {
      ChoreoTrajectory.createTrajectory("3_Piece_Mid_ADJUSTED")
    }
}
