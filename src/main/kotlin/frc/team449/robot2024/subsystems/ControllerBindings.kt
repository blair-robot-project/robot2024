package frc.team449.robot2024.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import kotlin.math.PI

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  private fun robotBindings() {
    mechanismController.rightBumper().onTrue(
      robot.undertaker.intake()
    ).onFalse(
      robot.undertaker.stop()
    )

    mechanismController.leftBumper().onTrue(
      robot.undertaker.outtake()
    ).onFalse(
      robot.undertaker.stop()
    )
  }

  private fun evergreenBindings() {
    // slow drive
    driveController.rightTrigger(0.75).onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 4 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
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
    evergreenBindings()
    robotBindings()
  }
}
