package frc.team449.robot2024.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.commands.driveAlign.OrbitAlign
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  val sysIdRoutine = SysIdRoutine(
    SysIdRoutine.Config(),
    Mechanism(
      { voltage: Measure<Voltage> -> robot.shooter.setLeftVoltage(voltage.`in`(Volts)) },
      null,
      robot.drive
    )
  )

  val orbitCmd = OrbitAlign(
    robot.drive,
    robot.driveController.hid,
    FieldConstants.SUBWOOFER_POSE
  )

  private fun stopAll(): Command {
    return ParallelCommandGroup(
      robot.undertaker.stop(),
      robot.shooter.coast(),
      robot.feeder.stop()
    )
  }

  private fun robotBindings() {
    mechanismController.y().onTrue(
      robot.pivot.moveAmp()
    )

    mechanismController.a().onTrue(
      robot.pivot.moveStow()
    )

    mechanismController.x().onTrue(
      robot.shooter.scoreAmp().alongWith(
        robot.feeder.intake()
      )
    ).onFalse(
      stopAll()
    )

    driveController.leftTrigger().onTrue(
      ParallelCommandGroup(
        robot.feeder.outtake(),
        robot.shooter.duringIntake(),
        robot.undertaker.outtake()
      )
    ).onFalse(
      stopAll()
    )

    mechanismController.b().onTrue(
      robot.shooter.stop()
    ).onFalse(
      stopAll()
    )

    driveController.rightTrigger().onTrue(
      intakePiece().andThen(
        outtakeToNotePosition()
      )
    ).onFalse(
      outtakeToNotePosition()
    )

    mechanismController.leftBumper().onTrue(
      ParallelCommandGroup(
        robot.feeder.outtake(),
        robot.shooter.duringIntake()
      )
    ).onFalse(
      stopAll()
    )

    mechanismController.rightBumper().onTrue(
      SequentialCommandGroup(
        outtakeToNotePosition(),
        robot.shooter.shootSubwoofer()
      )
    )

    mechanismController.rightTrigger().onTrue(
      SequentialCommandGroup(
        WaitUntilCommand { robot.shooter.atSetpoint() },
        robot.feeder.intake(),
        robot.undertaker.intake()
      ).alongWith(
        robot.shooter.shootSubwoofer()
      )
    ).onFalse(
      SequentialCommandGroup(
        stopAll(),
        robot.shooter.stop()
      )
    )

//    /** Characterization */
//    // Quasistatic Forwards
//    driveController.povUp().onTrue(
//      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
//    )
//
//    // Quasistatic Reverse
//    driveController.povDown().onTrue(
//      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
//    )
//
//    // Dynamic Forwards
//    driveController.povRight().onTrue(
//      sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
//    )
//
//    // Dynamic Reverse
//    driveController.povLeft().onTrue(
//      sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
//    )

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
  }

  private fun intakePiece(): Command {
    return SequentialCommandGroup(
      robot.undertaker.intake(),
      robot.feeder.intake(),
      WaitUntilCommand { !robot.infrared.get() },
      stopAll()
    )
  }

  private fun outtakeToNotePosition(): Command {
    return ConditionalCommand(
      SequentialCommandGroup(
        robot.undertaker.stop(),
        robot.feeder.outtake(),
        robot.shooter.duringIntake(),
        WaitUntilCommand { robot.infrared.get() },
        robot.feeder.stop(),
        robot.shooter.coast()
      ),
      InstantCommand()
    ) { !robot.infrared.get() }
  }

  private fun nonRobotBindings() {
    // slow drive
    driveController.rightBumper().onTrue(
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
      ConditionalCommand(
        InstantCommand({ robot.drive.heading = Rotation2d(PI) }),
        InstantCommand({ robot.drive.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
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
