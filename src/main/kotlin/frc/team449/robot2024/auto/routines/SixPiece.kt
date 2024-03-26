package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import kotlin.math.abs

class SixPiece(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.146901845932007, 4.836400508880615))))))
        ),
        1 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.55, 5.525))))))
        ),
        2 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.633739948272705, 6.8354172706604))))))
        ),
        3 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.983095407485962, 6.08734130859375))))))
        ),
        4 to AutoUtil.autoFarIntake(
          robot,
          Units.degreesToRadians(SpinShooterConstants.equation(Units.metersToInches(abs(FieldConstants.BLUE_SPEAKER_POSE.getDistance(Translation2d(2.983095407485962, 6.08734130859375))))))
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
        ChoreoTrajectory.createTrajectory("6_Piece")
      )
    } else {
      ChoreoTrajectory.createTrajectory("6_Piece")
    }
}
