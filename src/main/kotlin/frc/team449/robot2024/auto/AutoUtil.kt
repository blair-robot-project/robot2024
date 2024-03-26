package frc.team449.robot2024.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.auto.AutoConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

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
    return ConditionalCommand(
      ParallelDeadlineGroup(
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
      ).andThen(PrintCommand("!!!!!!!!!!!!!!FINISHED AUTO SHOOT!!!!!!!!!!!")),
      ParallelDeadlineGroup(
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
      ).andThen(PrintCommand("!!!!!!!!!!!!!!FINISHED AUTO SHOOT!!!!!!!!!!!")).withTimeout(0.95)
    ) { RobotBase.isReal() }
  }

  fun autoFarIntake(robot: Robot, angle: Double): Command {
    return ConditionalCommand(
      ParallelCommandGroup(
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
        robot.shooter.shootAnywhere(),
        SequentialCommandGroup(
          robot.pivot.moveStow().until { !robot.infrared.get() },
          robot.pivot.moveAngleCmd(angle)
        )
      ),
      ParallelCommandGroup(
        SequentialCommandGroup(
          robot.undertaker.intake(),
          robot.feeder.intake(),
          WaitCommand(1.0),
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
        robot.shooter.shootAnywhere(),
        SequentialCommandGroup(
          robot.pivot.moveStow().withTimeout(1.85),
          robot.pivot.moveAngleCmd(angle)
        )
      )
    ) { RobotBase.isReal() }
  }
  fun autoFarShoot(robot: Robot): Command {
    val cmd = ConditionalCommand(
      FunctionalCommand(
        { },
        {
          robot.shooter.shootPiece(
            SpinShooterConstants.ANYWHERE_LEFT_SPEED,
            SpinShooterConstants.ANYWHERE_RIGHT_SPEED
          )

          val distance = Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation)))

          val angle = Units.degreesToRadians(SpinShooterConstants.equation(distance))

          robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

          val fieldToRobot = robot.drive.pose.translation
          val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot

          robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

          if (robot.shooter.atSetpoint() &&
            abs((robot.drive.heading - robotToPoint.angle).radians) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
            robot.pivot.inTolerance()
          ) {
            robot.feeder.intakeVoltage()
            robot.undertaker.intakeVoltage()
          }
        },
        { },
        { robot.infrared.get() && robot.closeToShooterInfrared.get() },
        robot.shooter,
        robot.pivot,
        robot.feeder,
        robot.undertaker
      ),
      FunctionalCommand(
        { },
        {
          robot.shooter.shootPiece(
            SpinShooterConstants.ANYWHERE_LEFT_SPEED,
            SpinShooterConstants.ANYWHERE_RIGHT_SPEED
          )

          val distance = Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation)))

          val angle = Units.degreesToRadians(SpinShooterConstants.equation(distance))

          robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

          val fieldToRobot = robot.drive.pose.translation
          val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot

          robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

          if (robot.shooter.atSetpoint() &&
            abs((robot.drive.heading - robotToPoint.angle).radians) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
            robot.pivot.inTolerance()
          ) {
            robot.feeder.intakeVoltage()
            robot.undertaker.intakeVoltage()
          }
        },
        { },
        { false },
        robot.shooter,
        robot.pivot,
        robot.feeder,
        robot.undertaker
      ).withTimeout(0.40)
    ) { RobotBase.isReal() }
    cmd.name = "auto aiming"
    return cmd
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
