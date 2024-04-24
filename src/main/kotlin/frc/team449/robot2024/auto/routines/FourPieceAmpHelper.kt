package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import frc.team449.robot2024.constants.vision.VisionConstants

class FourPieceAmpHelper(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  val shot1PivotAngle = Units.degreesToRadians(
    SpinShooterConstants.SHOOTING_MAP.get(
      Units.metersToInches(
        FieldConstants.BLUE_SPEAKER_POSE.getDistance(
          Translation2d(
            3.7956974506378174,
            6.5
          )
        )
      )
    )
  )

  val shot2PivotAngle = Units.degreesToRadians(
    SpinShooterConstants.SHOOTING_MAP.get(
      Units.metersToInches(
        FieldConstants.BLUE_SPEAKER_POSE.getDistance(
          Translation2d(
            3.7956974506378174,
            6.5
          )
        )
      )
    )
  )

  val shot3PivotAngle = Units.degreesToRadians(
    SpinShooterConstants.SHOOTING_MAP.get(
      Units.metersToInches(
        FieldConstants.BLUE_SPEAKER_POSE.getDistance(
          Translation2d(
            3.7956974506378174,
            6.5
          )
        )
      )
    )
  )

  override val routine =
    ChoreoRoutine(
      thetaController = PIDController(2.85, 0.0, 0.075),
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoFarIntakeV2PremoveQuick(
          robot,
          shot1PivotAngle
        ),
        1 to AutoUtil.autoFarIntakeV2PremoveQuick(
          robot,
          shot2PivotAngle
        ),
        2 to ParallelCommandGroup(
          robot.pivot.moveStow(),
          robot.shooter.shootSubwoofer(),
          robot.feeder.intake(),
          robot.undertaker.intake()
        ),
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot).andThen(
          InstantCommand({
            robot.drive.enableVisionFusion = true
            VisionConstants.MAX_DISTANCE_SINGLE_TAG = 4.5
            VisionConstants.SINGLE_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .325, .325, 3.0))
            VisionConstants.MULTI_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .225, .225, 3.0))
          })
        ),
        1 to AutoUtil.autoFarShootHelperVisionSlow(robot, offset = Units.degreesToRadians(2.80)),
        2 to AutoUtil.autoFarShootHelperVisionSlow(robot, offset = Units.degreesToRadians(-0.375)),
        3 to SequentialCommandGroup(
          InstantCommand({
            robot.drive.enableVisionFusion = true
            VisionConstants.MAX_DISTANCE_SINGLE_TAG = 4.5
            VisionConstants.SINGLE_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .15, .15, 3.0))
            VisionConstants.MULTI_TAG_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .10, .10, 3.0))
          }),
          InstantCommand({ robot.drive.stop() }),
          WaitCommand(0.50),
          robot.undertaker.stop(),
          robot.feeder.stop(),
          robot.shooter.forceStop(),
          robot.pivot.moveStow()
        )
      ),
      debug = false,
      timeout = 2.15
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("Amp_Side")
      )
    } else {
      ChoreoTrajectory.createTrajectory("Amp_Side")
    }
}
