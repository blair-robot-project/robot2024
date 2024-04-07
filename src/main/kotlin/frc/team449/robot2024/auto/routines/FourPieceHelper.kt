package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
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

  val speakerPose = if (isRed) FieldConstants.RED_SPEAKER_POSE else FieldConstants.BLUE_SPEAKER_POSE

  val shot0Pose = Translation2d(1.443, 4.100571155548096)
  val shot0Offset = -0.085 // -0.5

  val shot1Pose = Translation2d(3.075 + 0.25, 2.90)
  val shot1Offset = -8.35 // -7.825 then -7.945 then 8.1

  val shot2Pose = Translation2d(3.25 + 0.25, 2.95)
  val shot2Offset = -7.325 // -7.2 then -7.14

  val shot3Pose = Translation2d(3.565, 3.0)
  val shot3Offset = -3.635 // -4.45

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoFarIntakeV2(robot),
        1 to AutoUtil.autoFarIntakeV2(
          robot,
//          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(shot2Pose)))) + shot2Offset)
        ),
        2 to AutoUtil.autoFarIntakeV2(
          robot,
//          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(shot3Pose)))) + shot3Offset)
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
          AutoUtil.autoFarShootHelperV2(robot, shot0Pose, offset = shot0Offset)
        ),
        1 to AutoUtil.autoFarShootHelperV2(robot, shot1Pose, offset = shot1Offset),
        2 to AutoUtil.autoFarShootHelperV2(robot, shot2Pose, offset = shot2Offset),
        3 to AutoUtil.autoFarShootHelperV2(robot, shot3Pose, offset = shot3Offset).andThen(
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
