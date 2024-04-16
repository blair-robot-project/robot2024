package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.controller.PIDController
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
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants

class FourPieceAmpHelper(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  // TODO: FINISH THIS UP

  val shot1DriveAngle = Units.degreesToRadians(12.18771219226771)
  val shot1PivotAngle = Units.degreesToRadians(
    SpinShooterConstants.equation(
      Units.metersToInches(
        FieldConstants.BLUE_SPEAKER_POSE.getDistance(
          Translation2d(
            3.7956974506378174,
            6.5
          )
        )
      )
    ) - 0.5
  )

  val shot2DriveAngle = Units.degreesToRadians(12.18771219226771)
  val shot2PivotAngle = Units.degreesToRadians(
    SpinShooterConstants.equation(
      Units.metersToInches(
        FieldConstants.BLUE_SPEAKER_POSE.getDistance(
          Translation2d(
            3.7956974506378174,
            6.5
          )
        )
      )
    ) - 0.5
  )

  val shot3DriveAngle = Units.degreesToRadians(12.18771219226771)
  val shot3PivotAngle = Units.degreesToRadians(
    SpinShooterConstants.equation(
      Units.metersToInches(
        FieldConstants.BLUE_SPEAKER_POSE.getDistance(
          Translation2d(
            3.7956974506378174,
            6.5
          )
        )
      )
    ) - 0.5
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
        2 to AutoUtil.autoFarIntakeV2PremoveQuick(
          robot,
          shot3PivotAngle
        ),
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoFarShootHelperV2(robot, shot1DriveAngle, shot1PivotAngle, fast = false),
        2 to AutoUtil.autoFarShootHelperV2(robot, shot2DriveAngle, shot2PivotAngle, fast = false),
        3 to AutoUtil.autoFarShootHelperV2(robot, shot3DriveAngle, shot3PivotAngle, fast = false)
          .andThen(
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
        ChoreoTrajectory.createTrajectory("Amp_Side")
      )
    } else {
      ChoreoTrajectory.createTrajectory("Amp_Side")
    }
}
