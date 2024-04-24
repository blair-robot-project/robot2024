package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.math.PI

class FourPieceHelper(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  val shot0DriveAngle = Units.degreesToRadians(-44.338651066156395)
  val shot0PivotAngle = Units.degreesToRadians(14.48705441278648)

  val shot1PivotAngle = Units.degreesToRadians(26.242354379317156)

  val shot2PivotAngle = Units.degreesToRadians(27.88355832196282)

  val shot3PivotAngle = Units.degreesToRadians(31.705604274195668)

  override val routine =
    ChoreoRoutine(
      thetaController = PIDController(2.85, 0.0, 0.075),
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoFarIntakeV2PremoveCenterline(
          robot,
          shot1PivotAngle,
          3.25
        ),
        1 to AutoUtil.autoFarIntakeV2Premove(
          robot,
          shot2PivotAngle
        ),
        2 to AutoUtil.autoFarIntakeV2Premove(
          robot,
          shot3PivotAngle
        ),
      ),
      stopEventMap = hashMapOf(
        0 to SequentialCommandGroup(
          if (isRed) {
            InstantCommand({ robot.drive.pose = Pose2d(FieldConstants.fieldLength - 1.443, 4.100571155548096, Rotation2d(PI)) })
          } else {
            InstantCommand(
              { robot.drive.pose = Pose2d(1.443, 4.100571155548096, Rotation2d()) }
            )
          },
          AutoUtil.autoFarShootHelperV2(robot, shot0DriveAngle, shot0PivotAngle, fast = true)
        ),
        1 to AutoUtil.autoJiggle(robot).andThen(
          AutoUtil.autoFarShootHelperVisionSlow(robot)
        ),
        2 to AutoUtil.autoJiggle(robot).andThen(
          AutoUtil.autoFarShootHelperVisionSlow(robot)
        ),
        3 to AutoUtil.autoJiggle(robot).andThen(
          AutoUtil.autoFarShootHelperVisionSlow(robot),
          InstantCommand({ robot.drive.stop() }),
          robot.undertaker.stop(),
          robot.feeder.stop(),
          WaitCommand(0.050),
          robot.shooter.forceStop(),
          robot.pivot.moveStow(),
        )
      ),
      debug = false,
      timeout = 0.0
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("4_Piece_Helper_MiddleDown")
      )
    } else {
      ChoreoTrajectory.createTrajectory("4_Piece_Helper_MiddleDown")
    }
}
