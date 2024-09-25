package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterKrakenConstants
import frc.team449.robot2024.constants.vision.VisionConstants

class Vision3PieceMid(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  private val shot1Offset = Units.degreesToRadians(0.0)
  private val shot2Offset = Units.degreesToRadians(0.0)

  private val shot1PivotAngle = SpinShooterKrakenConstants.SHOOTING_MAP.get(
    FieldConstants.BLUE_SPEAKER_POSE.getDistance(
      Translation2d(
        2.024296283721924,
        3.545278549194336
      )
    )
  ) - shot1Offset

  private val shot2PivotAngle = SpinShooterKrakenConstants.SHOOTING_MAP.get(
    FieldConstants.BLUE_SPEAKER_POSE.getDistance(
      Translation2d(
        2.024296283721924,
        3.545278549194336
      )
    )
  ) - shot2Offset

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoFarIntakeV2PremoveQuick(
          robot,
          shot1PivotAngle
        ),
        1 to AutoUtil.autoFarIntakeV2PremoveQuick(
          robot,
          shot2PivotAngle
        )
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot).andThen(
          InstantCommand({
            robot.drive.enableVisionFusion = true
            VisionConstants.MAX_DISTANCE_SINGLE_TAG = 0.1
          })
        ),
        1 to AutoUtil.autoShoot(robot),
        2 to AutoUtil.autoShoot(robot)
          .andThen(
            InstantCommand({
              VisionConstants.MAX_DISTANCE_SINGLE_TAG = 5.0
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
        ChoreoTrajectory.createTrajectory("3_Piece_Mid_VISION")
      )
    } else {
      ChoreoTrajectory.createTrajectory("3_Piece_Mid_VISION")
    }
}
