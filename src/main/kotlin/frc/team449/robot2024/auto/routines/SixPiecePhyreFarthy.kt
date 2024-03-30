package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import kotlin.math.abs

class SixPiecePhyreFarthy(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.55, 5.558067321777344))))))
        ),
        1 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.6177337169647217, 6.778044700622559))))))
        ),
        2 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.8952739238739014, 5.56283712387085))))))
        ),
        3 to AutoUtil.autoFarIntakeCenterline(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.521202325820923, 5.297963619232178)))))),
          3.45
        ),
        4 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.194774866104126, 4.563653469085693))))))
        )
      ),
      stopEventMap = hashMapOf(
        0 to AutoUtil.autoShoot(robot),
        1 to AutoUtil.autoFarShoot(robot),
        2 to AutoUtil.autoFarShoot(robot),
        3 to AutoUtil.autoFarShoot(robot),
        4 to AutoUtil.autoFarShoot(robot),
        5 to AutoUtil.autoFarShoot(robot).andThen(
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
        ChoreoTrajectory.createTrajectory("6_Piece_Phyre_Farthy")
      )
    } else {
      ChoreoTrajectory.createTrajectory("6_Piece_Phyre_Farthy")
    }
}
