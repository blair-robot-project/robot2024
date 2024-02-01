package frc.team449.robot2024.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import kotlin.math.PI

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  private fun robotBindings() {
    //Press right bumper to suck up, and feed.
    JoystickButton(driveController, XboxController.Button.kRightBumper.value).onTrue(
      robot.undertaker.intake()
    ).onFalse(
      robot.undertaker.stop()
    )
    JoystickButton(driveController, XboxController.Button.kRightBumper.value).onTrue(
      robot.feeder.intake()
    ).onFalse(
      robot.feeder.stop()
    )
    JoystickButton(driveController, XboxController.Button.kRightBumper.value).onTrue(
      robot.shooter.duringIntake()
    ).onFalse(
      robot.shooter.stopIntake()
    )
    //Press left bumper to let out, and un-feed.
    JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
      robot.undertaker.outtake()
    ).onFalse(
      robot.undertaker.stop()
    )
    JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
      robot.feeder.outtake()
    ).onFalse(
      robot.feeder.stop()
    )

    //yes
    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
      robot.shooter.shootSubwoofer()
    )
    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
      robot.shooter.scoreAmp()
    )
    JoystickButton(driveController, XboxController.Button.kX.value).onTrue(
      robot.shooter.shootAnywhere()
    )
    JoystickButton(driveController, XboxController.Button.kY.value).onTrue(
      robot.pivot.hold()
    )
  }

  private fun evergreenBindings() {
    // slow drive
    Trigger { driveController.rightTriggerAxis >= .75 }.onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 }).andThen(
        InstantCommand({ robot.drive.maxRotSpeed = PI / 4 })
      )
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED }).andThen(
        InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
      )
    )

    // reset gyro
    JoystickButton(driveController, XboxController.Button.kStart.value).onTrue(
      InstantCommand({
        robot.drive.heading = Rotation2d()
      })
    )

    // introduce "noise" to the simulated pose
    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
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
