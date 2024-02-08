package frc.team449.robot2024.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.commands.driveAlign.OrbitAlign
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.math.PI

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  val orbitCmd = OrbitAlign(
    robot.drive,
    robot.driveController.hid,
    FieldConstants.SUBWOOFER_POSE
  )

  private fun robotBindings() {
    driveController.rightBumper().onTrue(
      if (!robot.infrared.get()) {
        ParallelCommandGroup(
          robot.undertaker.intake(),
          robot.feeder.intake(),
          robot.shooter.duringIntake()
        )
      } else {
        ParallelCommandGroup(
          robot.undertaker.stop(),
          robot.feeder.stop(),
          robot.shooter.stop()
        )
      }
    ).onFalse(
      ParallelCommandGroup(
        robot.undertaker.stop(),
        robot.feeder.stop(),
        robot.shooter.stop()
      )
    )
    driveController.leftBumper().onTrue(
      ParallelCommandGroup(
        robot.undertaker.outtake(),
        robot.feeder.outtake(),
        robot.shooter.duringIntake()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.undertaker.stop(),
        robot.feeder.stop(),
        robot.shooter.stop()
      )
    )

    driveController.leftBumper().onTrue(
      robot.shooter.shootSubwoofer()
    ).onFalse(
      robot.shooter.stop()
    )

    /** Shooting from anywhere */
//    mechanismController.b().onTrue(
//      ParallelCommandGroup(
//        orbitCmd,
//        robot.shooter.shootAnywhere(),
//        robot.pivot.pivotShootAnywhere(),
//        SequentialCommandGroup(
//          WaitUntilCommand {
//            orbitCmd.atSetpoint() &&
//              robot.shooter.atSetpoint() &&
//              robot.pivot.atSetpoint()
//          },
//          PrintCommand("GOING TO SHOOT!!!"),
//          robot.undertaker.intake(),
//          robot.feeder.intake()
//        )
//      )
//    )

//    mechanismController.b().onTrue(
//      robot.pivot()
//    )

    mechanismController.x().onTrue(
      PrintCommand("whats good chat").andThen(robot.feeder.intake())
    ).onFalse(
      robot.feeder.stop()
    )

    mechanismController.a().onTrue(
      robot.feeder.intake().andThen(robot.shooter.scoreAmp())
    ).onFalse(
      robot.feeder.stop().andThen(robot.shooter.stop())
    )
  }

  private fun nonRobotBindings() {
    // slow drive
    driveController.rightTrigger(0.5).onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 4 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(
          InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
        )
    )

    // reset gyro
    driveController.start().onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d() })
    )

    // introduce "noise" to the simulated pose
    driveController.b().onTrue(
      ConditionalCommand(
        InstantCommand({
          robot.drive as SwerveSim
          robot.drive.resetPos()
        }),
        InstantCommand()
      ) { RobotBase.isSimulation() }
    )
  }

  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
