package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class SixPiece(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  private val farShot1PivotAngle = Units.degreesToRadians(24.924282375190465)
  private val farShot1DriveAngle = Units.degreesToRadians(27.699505291327124)

  private val farShot2PivotAngle = Units.degreesToRadians(23.48886092186779)
  private val farShot2DriveAngle = Units.degreesToRadians(13.49573328082929)

  private val farShot3PivotAngle = Units.degreesToRadians(23.48886092186779)
  private val farShot3DriveAngle = Units.degreesToRadians(13.49573328082929)

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to SequentialCommandGroup(
          AutoUtil.autoIntake(robot).withTimeout(1.85),
          AutoUtil.autoShootInMotion(robot),
          AutoUtil.autoIntake(robot).withTimeout(3.55 - 2.05),
          AutoUtil.autoShootInMotion(robot),
          ParallelCommandGroup(
            SequentialCommandGroup(
              robot.undertaker.intake(),
              robot.feeder.slowIntake(),
              WaitUntilCommand { !robot.infrared.get() },
              robot.undertaker.stop(),
              ConditionalCommand(
                SequentialCommandGroup(
                  robot.feeder.outtake(),
                  WaitUntilCommand { !robot.closeToShooterInfrared.get() },
                  robot.feeder.stop()
                ),
                SequentialCommandGroup(
                  robot.feeder.outtake(),
                  WaitCommand(0.065),
                  robot.feeder.stop()
                ),
              ) { !robot.closeToShooterInfrared.get() }
            ),
            robot.shooter.shootAnywhere()
          )
        ),
        1 to AutoUtil.autoFarIntakeV2Premove(
          robot,
          farShot2PivotAngle
        ),
        2 to AutoUtil.autoFarIntakeV2Premove(
          robot,
          farShot3PivotAngle
        ),
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoFarShootHelperV2(robot, farShot1DriveAngle, farShot1PivotAngle),
        2 to AutoUtil.autoFarShootHelperV2(robot, farShot2DriveAngle, farShot2PivotAngle),
        3 to AutoUtil.autoFarShootHelperV2(robot, farShot3DriveAngle, farShot3PivotAngle).andThen(
          InstantCommand({ robot.drive.stop() }),
          robot.undertaker.stop(),
          robot.feeder.stop(),
          WaitCommand(0.050),
          robot.shooter.forceStop(),
          robot.pivot.moveStow()
        )
      ),
      debug = false,
      timeout = 0.0
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("6_Piece_V2")
      )
    } else {
      ChoreoTrajectory.createTrajectory("6_Piece_V2")
    }
}
