package frc.team449.robot2024.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Rotation2d
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
import frc.team449.robot2024.constants.subsystem.FeederConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

object AutoUtil {

  private fun autoJiggle(robot: Robot): Command {
    return SequentialCommandGroup(
      ConditionalCommand(
        SequentialCommandGroup(
          ParallelCommandGroup(
            robot.undertaker.slowIntake(),
            robot.feeder.slowIntake(),
          ),
          WaitUntilCommand { !robot.closeToShooterInfrared.get() }
        ),
        InstantCommand()
      ) { robot.closeToShooterInfrared.get() },
      SequentialCommandGroup(
        robot.undertaker.stop(),
        robot.feeder.outtake(),
        WaitUntilCommand { robot.closeToShooterInfrared.get() },
        robot.undertaker.stop().alongWith(
          robot.feeder.stop()
        )
      )
    )
  }

  fun autoIntake(robot: Robot): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.undertaker.intake(),
        robot.feeder.slowIntake(),
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
            WaitCommand(0.065),
            robot.feeder.stop()
          ),
        ) { !robot.closeToShooterInfrared.get() }
      ),
      robot.shooter.shootSubwoofer()
    )
  }

  fun autoIntakeCenterline(robot: Robot): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.undertaker.intake(),
        robot.feeder.slowIntake(),
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
            WaitCommand(0.065),
            robot.feeder.stop()
          ),
        ) { !robot.closeToShooterInfrared.get() },
        autoJiggle(robot),
        autoJiggle(robot),
        autoJiggle(robot),
        autoJiggle(robot)
      ),
      robot.shooter.shootSubwoofer()
    )
  }

  fun autoIntakePass(robot: Robot): Command {
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
          robot.feeder.stop(),
        ) { !robot.closeToShooterInfrared.get() }
      ),
      robot.shooter.shootPass()
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

  fun autoShootInMotion(robot: Robot): Command {
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
      ).andThen(PrintCommand("!!!!!!!!!!!!!!FINISHED AUTO SHOOT!!!!!!!!!!!")).withTimeout(0.20)
    ) { RobotBase.isReal() }
  }

  fun autoPass(robot: Robot): Command {
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
        robot.shooter.shootPass(),
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
          robot.feeder.slowIntake(),
          WaitUntilCommand { !robot.infrared.get() },
          robot.undertaker.stop(),
          ConditionalCommand(
            SequentialCommandGroup(
              robot.feeder.outtake(),
              WaitUntilCommand { !robot.closeToShooterInfrared.get() },
              robot.feeder.stop()
            ),
            robot.feeder.stop()
          ) { !robot.closeToShooterInfrared.get() }
        ),
        robot.shooter.shootAnywhere(),
        SequentialCommandGroup(
          robot.pivot.moveStow()
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
            robot.feeder.stop()
          ) { !robot.closeToShooterInfrared.get() }
        ),
        robot.shooter.shootAnywhere(),
        SequentialCommandGroup(
          robot.pivot.moveStow()
        )
      )
    ) { RobotBase.isReal() }
  }

  fun autoFarIntakeCenterline(robot: Robot, angle: Double, waitTime: Double): Command {
    return ConditionalCommand(
      ParallelCommandGroup(
        SequentialCommandGroup(
          robot.undertaker.intake(),
          robot.feeder.slowIntake(),
          WaitUntilCommand { !robot.infrared.get() },
          robot.undertaker.stop(),
          ConditionalCommand(
            SequentialCommandGroup(
              robot.feeder.outtake(),
              WaitUntilCommand { !robot.closeToShooterInfrared.get() },
              robot.feeder.stop()
            ),
            robot.feeder.stop()
          ) { !robot.closeToShooterInfrared.get() }
        ),
        robot.shooter.shootAnywhere(),
        SequentialCommandGroup(
          robot.pivot.moveStow()
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
            robot.feeder.stop()
          ) { !robot.closeToShooterInfrared.get() }
        ),
        robot.shooter.shootAnywhere(),
        SequentialCommandGroup(
          robot.pivot.moveStow()
        )
      )
    ) { RobotBase.isReal() }
  }

  fun autoFarIntakeV2(robot: Robot): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.undertaker.intake(),
        robot.feeder.slowIntake(),
        WaitUntilCommand { !robot.infrared.get() },
        autoJiggle(robot),
        autoJiggle(robot),
        autoJiggle(robot),
        autoJiggle(robot)
      ),
      SequentialCommandGroup(
        robot.shooter.shootAnywhere()
      ),
      SequentialCommandGroup(
        robot.pivot.moveStow()
      )
    )
  }

  fun autoFarIntakeV2PremoveQuick(robot: Robot, angle: Double): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.pivot.moveStow().until { robot.pivot.inTolerance(PivotConstants.AMP_ANGLE) }.andThen(
          InstantCommand({ robot.pivot.setVoltage(-0.35) })
        ),
        robot.undertaker.intake(),
        robot.feeder.slowIntake(),
        WaitUntilCommand { !robot.infrared.get() },
        autoJiggle(robot),
        ParallelCommandGroup(
          SequentialCommandGroup(
            autoJiggle(robot),
            autoJiggle(robot)
          ),
          robot.pivot.moveAngleCmdAuto(angle)
        ),
      ),
      SequentialCommandGroup(
        robot.shooter.shootAnywhere()
      )
    )
  }

  fun autoFarIntakeV2Premove(robot: Robot, angle: Double): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.pivot.moveStow().until { robot.pivot.inTolerance(PivotConstants.AMP_ANGLE) }.andThen(
          InstantCommand({ robot.pivot.setVoltage(-0.35) })
        ),
        robot.undertaker.intake(),
        robot.feeder.slowIntake(),
        WaitUntilCommand { !robot.infrared.get() },
        autoJiggle(robot),
        autoJiggle(robot),
        ParallelCommandGroup(
          SequentialCommandGroup(
            autoJiggle(robot),
            autoJiggle(robot),
            autoJiggle(robot),
            autoJiggle(robot)
          ),
          robot.pivot.moveAngleCmdAuto(angle)
        ),
      ),
      SequentialCommandGroup(
        robot.shooter.shootAnywhere()
      )
    )
  }

  fun autoFarIntakeV2PremoveCenterline(robot: Robot, angle: Double, waitTime: Double): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.pivot.moveStow().until { robot.pivot.inTolerance(PivotConstants.AMP_ANGLE) }.andThen(
          InstantCommand({ robot.pivot.setVoltage(-0.35) })
        ),
        robot.undertaker.intake(),
        robot.feeder.slowIntake(),
        WaitUntilCommand { !robot.infrared.get() },
        autoJiggle(robot),
        autoJiggle(robot),
        autoJiggle(robot),
        autoJiggle(robot),
        WaitCommand(waitTime),
        ParallelCommandGroup(
          SequentialCommandGroup(
            autoJiggle(robot),
            autoJiggle(robot)
          ),
          robot.pivot.moveAngleCmdAuto(angle)
        )
      ),
      SequentialCommandGroup(
        robot.shooter.shootAnywhere()
      )
    )
  }

  fun autoFarShootHelperV2(robot: Robot, driveAngle: Double, pivotAngle: Double, fast: Boolean = false): Command {
    val cmd = SequentialCommandGroup(
      FunctionalCommand(
        {
          val robotAngle = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
            MathUtil.angleModulus(PI - driveAngle)
          } else {
            MathUtil.angleModulus(driveAngle)
          }

          RobotConstants.ORTHOGONAL_CONTROLLER.setpoint = robotAngle
        },
        {
          val robotAngle = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
            MathUtil.angleModulus(PI - driveAngle)
          } else {
            MathUtil.angleModulus(driveAngle)
          }

          robot.shooter.shootPiece(
            SpinShooterConstants.ANYWHERE_LEFT_SPEED,
            SpinShooterConstants.ANYWHERE_RIGHT_SPEED
          )

          if (fast) {
            robot.pivot.moveToAngleSlow(MathUtil.clamp(pivotAngle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))
          } else {
            robot.pivot.moveToAngleAuto(MathUtil.clamp(pivotAngle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))
          }

          robot.drive.set(
            ChassisSpeeds(
              0.0,
              0.0,
              RobotConstants.ORTHOGONAL_CONTROLLER.calculate(
                robot.drive.heading.radians
              )
            )
          )

          if (robot.shooter.atSetpoint() &&
            abs(MathUtil.angleModulus(robot.drive.heading.radians - robotAngle)) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
            (robot.pivot.inAutoTolerance(pivotAngle) && !fast) ||
            (fast && robot.pivot.inTolerance(pivotAngle))
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
      )
    )
    cmd.name = "auto aiming"
    return cmd
  }

  fun autoFarShootHelperVision(robot: Robot, fast: Boolean = false): Command {
    val cmd = SequentialCommandGroup(
      FunctionalCommand(
        { },
        {
          robot.shooter.shootPiece(
            SpinShooterConstants.ANYWHERE_LEFT_SPEED,
            SpinShooterConstants.ANYWHERE_RIGHT_SPEED
          )

          val distance = abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation))

          val pivotAngle = if (distance <= 1.30) 0.0 else SpinShooterConstants.SHOOTING_MAP.get(distance)

          if (fast) {
            robot.pivot.moveToAngleSlow(MathUtil.clamp(pivotAngle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))
          } else {
            robot.pivot.moveToAngleAuto(MathUtil.clamp(pivotAngle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))
          }

          val robotToPoint = FieldConstants.SPEAKER_POSE - robot.drive.pose.translation

          val desiredAngle = robotToPoint.angle + Rotation2d(PI)

          robot.drive.set(
            ChassisSpeeds(
              0.0,
              0.0,
              RobotConstants.ORTHOGONAL_CONTROLLER.calculate(
                robot.drive.heading.radians,
                desiredAngle.radians
              )
            )
          )

          if (robot.shooter.atSetpoint() &&
            abs(MathUtil.angleModulus(robot.drive.heading.radians - desiredAngle.radians)) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
            (robot.pivot.inAutoTolerance(pivotAngle) && !fast) ||
            (fast && robot.pivot.inTolerance(pivotAngle))
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
      )
    )
    cmd.name = "auto aiming"
    return cmd
  }

  fun autoFarShoot(robot: Robot, offset: Double = 0.0): Command {
    val cmd = ConditionalCommand(
      SequentialCommandGroup(
        checkNoteInLocation(robot),
        FunctionalCommand(
          {
            val fieldToRobot = robot.drive.pose.translation
            val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot
            RobotConstants.ORTHOGONAL_CONTROLLER.setpoint = robotToPoint.angle.radians + PI
          },
          {
            robot.shooter.shootPiece(
              SpinShooterConstants.ANYWHERE_LEFT_SPEED,
              SpinShooterConstants.ANYWHERE_RIGHT_SPEED
            )

            val distance = Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation)))

            val angle = Units.degreesToRadians(SpinShooterConstants.equation(distance) + offset)

            robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

            val fieldToRobot = robot.drive.pose.translation
            val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot

            robot.drive.set(
              ChassisSpeeds(
                0.0,
                0.0,
                RobotConstants.ORTHOGONAL_CONTROLLER.calculate(
                  robot.drive.heading.radians
                )
              )
            )

            println(
              "drive thing: " +
                if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
                  abs(MathUtil.angleModulus(robot.drive.heading.radians - robotToPoint.angle.radians + PI) - 2 * PI) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD
                } else {
                  abs(MathUtil.angleModulus(robot.drive.heading.radians - robotToPoint.angle.radians + PI) - PI) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD
                }
            )

            if (robot.shooter.atSetpoint() &&
//            abs(MathUtil.angleModulus(robot.drive.heading.radians - robotToPoint.angle.radians + PI) - 2 * PI) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
              robot.pivot.inTolerance(angle)
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
        )
      ),
      FunctionalCommand(
        {
          val fieldToRobot = robot.drive.pose.translation
          val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot
          RobotConstants.ORTHOGONAL_CONTROLLER.setpoint = robotToPoint.angle.radians + PI
        },
        {
          robot.shooter.shootPiece(
            SpinShooterConstants.ANYWHERE_LEFT_SPEED,
            SpinShooterConstants.ANYWHERE_RIGHT_SPEED
          )

          val distance = Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation)))

          val angle = Units.degreesToRadians(SpinShooterConstants.equation(distance) + offset)

          robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

          val fieldToRobot = robot.drive.pose.translation
          val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot

          robot.drive.set(
            ChassisSpeeds(
              0.0,
              0.0,
              RobotConstants.ORTHOGONAL_CONTROLLER.calculate(
                robot.drive.heading.radians
              )
            )
          )

          if (robot.shooter.atSetpoint() &&
            abs(MathUtil.angleModulus(robot.drive.heading.radians - robotToPoint.angle.radians + PI) - PI) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
            robot.pivot.inTolerance(angle)
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

  private fun checkNoteInLocation(robot: Robot): Command {
    return ConditionalCommand(
      stopIntake(robot),
      SequentialCommandGroup(
        slowIntake(robot),
        outtakeToNotePosition(robot)
      )
    ) { !robot.infrared.get() && robot.closeToShooterInfrared.get() }
      .withTimeout(FeederConstants.CHECK_NOTE_IN_LOCATION_TIMEOUT_SECONDS)
  }

  private fun stopIntake(robot: Robot): Command {
    return ParallelCommandGroup(
      robot.undertaker.stop(),
      robot.feeder.stop()
    )
  }

  private fun slowIntake(robot: Robot): Command {
    return ConditionalCommand(
      SequentialCommandGroup(
        ParallelCommandGroup(
          robot.undertaker.slowIntake(),
          robot.feeder.slowIntake(),
        ),
        WaitUntilCommand { !robot.closeToShooterInfrared.get() }
      ),
      InstantCommand()
    ) { robot.closeToShooterInfrared.get() }
  }

  private fun outtakeToNotePosition(robot: Robot): Command {
    val cmd = ConditionalCommand(
      SequentialCommandGroup(
        robot.undertaker.stop(),
        robot.feeder.outtake(),
        WaitUntilCommand { robot.closeToShooterInfrared.get() },
        stopIntake(robot)
      ),
      stopIntake(robot)
    ) { !robot.closeToShooterInfrared.get() }

    cmd.name = "outtake to note pos"
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
