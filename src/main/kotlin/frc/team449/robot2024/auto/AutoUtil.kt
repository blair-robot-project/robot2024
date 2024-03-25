package frc.team449.robot2024.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.auto.AutoConstants
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.math.PI

object AutoUtil {

  fun autoIntake(robot: Robot): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.undertaker.intake(),
        robot.feeder.intake(),
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
            WaitCommand(0.05),
            robot.feeder.stop()
          )
        ) { !robot.closeToShooterInfrared.get() }
      ),
      robot.shooter.shootSubwoofer()
    )
  }

  fun autoShoot(robot: Robot): Command {
    return ParallelDeadlineGroup(
      SequentialCommandGroup(
        WaitUntilCommand { robot.shooter.atAutoSetpoint() }.withTimeout(AutoConstants.AUTO_SPINUP_TIMEOUT_SECONDS),
        robot.feeder.autoShootIntake(),
        robot.undertaker.intake(),
        SequentialCommandGroup(
          WaitUntilCommand { !robot.infrared.get() || !robot.closeToShooterInfrared.get() },
          WaitUntilCommand { robot.infrared.get() && robot.closeToShooterInfrared.get() }
        ).withTimeout(AutoConstants.AUTO_SHOOT_TIMEOUT_SECONDS)
      ),
      robot.shooter.shootSubwoofer(),
      InstantCommand({ robot.drive.set(ChassisSpeeds()) })
    ).andThen(PrintCommand("!!!!!!!!!!!!!!FINISHED AUTO SHOOT!!!!!!!!!!!"))
  }

  fun transformForRed(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder.fill(
          Nat.N2(),
          Nat.N3(),
          FieldConstants.fieldLength - currentMatrix[0, 0],
          currentMatrix[0, 1],
          MathUtil.angleModulus(PI - currentMatrix[0, 2]),
          -currentMatrix[1, 0],
          currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  /** Add other methods that return commands that do groups of actions that are done
   * across different auto routines. For Charged UP, these methods were things such as
   * dropping a cone/cube, or getting in ground intake position, etc.
   */
}
