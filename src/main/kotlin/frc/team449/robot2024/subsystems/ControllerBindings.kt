package frc.team449.robot2024.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.util.characterization.Characterization
import kotlin.math.PI

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
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

    // characterize
    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
      Characterization(
        robot.drive,
        true,
        "swerve drive",
        robot.drive::setVoltage,
        robot.drive::getModuleVel
      ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    ).onFalse(
      robot.driveCommand
    )

    // introduce "noise" to the simulated pose
    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
      InstantCommand({
        if (RobotBase.isSimulation()) {
          robot.drive as SwerveSim
          robot.drive.resetPos()
        }
      })
    )
  }
}
