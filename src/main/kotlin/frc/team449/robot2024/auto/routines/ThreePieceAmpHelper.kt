package frc.team449.robot2024.auto.routines

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
import frc.team449.robot2024.constants.subsystem.SpinShooterKrakenConstants
import frc.team449.robot2024.constants.vision.VisionConstants

class ThreePieceAmpHelper(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  private val shot1Offset = Units.degreesToRadians(0.0)
  private val shot2Offset = Units.degreesToRadians(0.0)

  private val shot1PivotAngle = SpinShooterKrakenConstants.SHOOTING_MAP.get(
    FieldConstants.BLUE_SPEAKER_POSE.getDistance(
      Translation2d(
        3.85,
        5.584280490875244
      )
    )
  ) - shot1Offset

  private val shot2PivotAngle = SpinShooterKrakenConstants.SHOOTING_MAP.get(
    FieldConstants.BLUE_SPEAKER_POSE.getDistance(
      Translation2d(
        3.85,
        5.584280490875244
      )
    )
  ) - shot2Offset

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
            VisionConstants.MAX_DISTANCE_SINGLE_TAG = 1e-6
          })
        ),
        1 to AutoUtil.autoFarShootHelperVisionSlow(robot, offset = shot1Offset),
        2 to AutoUtil.autoFarShootHelperVisionSlow(robot, offset = shot2Offset),
        3 to SequentialCommandGroup(
          InstantCommand({
            robot.drive.stop()
            VisionConstants.MAX_DISTANCE_SINGLE_TAG = 5.0
          }, robot.drive),
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
