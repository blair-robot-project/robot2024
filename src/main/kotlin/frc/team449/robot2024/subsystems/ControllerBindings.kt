package frc.team449.robot2024.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
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

  private fun stopAll(): Command {
    return ParallelCommandGroup(
      robot.undertaker.stop(),
      robot.shooter.stop(),
      robot.feeder.stop()
    )
  }

  private fun robotBindings() {
    driveController.povUp().onTrue(
      robot.pivot.moveAmp()
    )

    driveController.povRight().onTrue(
      robot.shooter.scoreAmp().alongWith(
        robot.feeder.stop()
      )
    ).onFalse(
      stopAll()
    )

    driveController.rightBumper().onTrue(
      AutoUtil.autoIntake(robot)
    ).onFalse(
      stopAll()
    )

//    driveController.leftBumper().onTrue(
//      ParallelCommandGroup(
//        robot.undertaker.outtake(),
//        robot.feeder.outtake(),
//        robot.shooter.duringIntake()
//      )
//    ).onFalse(
//      ParallelCommandGroup(
//        robot.undertaker.stop(),
//        robot.feeder.stop(),
//        robot.shooter.stop()
//      )
//    )

    driveController.leftBumper().onTrue(
      ParallelCommandGroup(
        robot.shooter.shootSubwoofer(),
        SequentialCommandGroup(
          WaitUntilCommand { robot.shooter.atSetpoint() },
          robot.feeder.intake(),
          robot.undertaker.intake()
        )
      )
    ).onFalse(
      stopAll()
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
      robot.feeder.intake()
    ).onFalse(
      robot.feeder.stop()
    )

    mechanismController.a().onTrue(
      ParallelCommandGroup(
        robot.shooter.shootSubwoofer(),
        SequentialCommandGroup(
          WaitUntilCommand { robot.shooter.atSetpoint() },
          robot.feeder.intake(),
          robot.undertaker.intake()
        )
      )
    ).onFalse(
      ParallelCommandGroup(
        stopAll()
      )
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
