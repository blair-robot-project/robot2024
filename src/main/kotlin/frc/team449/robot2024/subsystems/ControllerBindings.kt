package frc.team449.robot2024.subsystems

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.commands.driveAlign.OrbitAlign
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.FeederConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  val leftSysID = SysIdRoutine(
    SysIdRoutine.Config(
      Volts.of(0.20).per(Units.Seconds.of(1.0)),
      Volts.of(3.0),
      Units.Seconds.of(20.0)
    ) { state -> SignalLogger.writeString("leftMotorState", state.toString()) },
    Mechanism(
      { voltage: Measure<Voltage> ->
        run { robot.shooter.setLeftVoltage(voltage.`in`(Volts)) }
      },
      null,
      robot.shooter,
      "shooter"
    )
  )

  private val rightSysIDCommand = SysIdRoutine(
    SysIdRoutine.Config(
      Volts.of(0.20).per(Units.Seconds.of(1.0)),
      Volts.of(3.0),
      Units.Seconds.of(20.0)
    ) { state -> SignalLogger.writeString("rightMotorState", state.toString()) },
    Mechanism(
      { voltage: Measure<Voltage> ->
        run {
          robot.shooter.setRightVoltage(voltage.`in`(Volts))
        }
      },
      null,
      robot.shooter,
      "shooter"
    )
  )

  val driveSysID = SysIdRoutine(
    SysIdRoutine.Config(),
    Mechanism(
      { voltage: Measure<Voltage> -> robot.drive.setVoltage(voltage.`in`(Volts)) },
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
      robot.shooter.rampStop(),
      robot.feeder.stop()
    )
  }

  private fun robotBindings() {
    mechanismController.y().onTrue(
      ParallelCommandGroup(
        robot.undertaker.slowIntake().andThen(
          WaitCommand(0.25),
          robot.pivot.moveAmp()
        ),
        robot.shooter.scoreAmp()
      )
    )

    Trigger { abs(mechanismController.hid.leftY) > 0.15 || abs(mechanismController.hid.rightY) > 0.15 }.onTrue(
      robot.climber.manual({
        MathUtil.applyDeadband(
          -mechanismController.hid.leftY,
          0.15,
          1.0
        )
      }, {
        MathUtil.applyDeadband(
          -mechanismController.hid.rightY,
          0.15,
          1.0
        )
      })
    ).onFalse(
      robot.climber.stop()
    )

    robot.mechController.povRight().onTrue(
      robot.climber.extend()
    ).onFalse(
      robot.climber.stop()
    )

    robot.mechController.povLeft().onTrue(
      robot.climber.retract()
    ).onFalse(
      robot.climber.stop()
    )

    robot.mechController.povUp().onTrue(
      robot.pivot.manualUp()
    ).onFalse(
      robot.pivot.hold()
    )

    robot.mechController.povDown().onTrue(
      robot.pivot.manualDown()
    ).onFalse(
      robot.pivot.hold()
    )

    mechanismController.a().onTrue(
      robot.pivot.moveStow()
    )

    mechanismController.leftBumper().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition(),
        robot.shooter.scoreAmp(),
      )
    )

    mechanismController.leftTrigger().onTrue(
      SequentialCommandGroup(
        WaitUntilCommand { robot.shooter.atSetpoint() },
        robot.feeder.intake(),
      ).alongWith(
        robot.shooter.scoreAmp()
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
      robot.shooter.forceStop()
    ).onFalse(
      stopAll()
    )

    driveController.rightTrigger().onTrue(
      SequentialCommandGroup(
        intakePiece(),
        slowIntake(),
        outtakeToNotePosition()
      )
    ).onFalse(
      SequentialCommandGroup(
        checkNoteInLocation(),
        stopAll()
      )
    )

    mechanismController.x().onTrue(
      robot.pivot.moveClimb()
    )

    mechanismController.back().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition()
      )
        .withTimeout(FeederConstants.CHECK_NOTE_IN_LOCATION_TIMEOUT_SECONDS)
    )

    mechanismController.start().onTrue(
      ParallelCommandGroup(
        robot.feeder.outtake(),
        robot.shooter.duringIntake()
      )
    ).onFalse(
      stopAll()
    )

    mechanismController.rightBumper().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition(),
        robot.shooter.shootSubwoofer(),
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
      stopAll()
    )
    /** Characterization */
//    // Quasistatic Forwards
//    driveController.povUp().onTrue(
//      leftSysIDCommand.quasistatic(SysIdRoutine.Direction.kForward)
//    )
//
//    // Quasistatic Reverse
//    driveController.povDown().onTrue(
//      leftSysIDCommand.quasistatic(SysIdRoutine.Direction.kReverse)
//    )
//
//    // Dynamic Forwards
//    driveController.povRight().onTrue(
//      leftSysIDCommand.dynamic(SysIdRoutine.Direction.kForward)
//    )
//
//    // Dynamic Reverse
//    driveController.povLeft().onTrue(
//      leftSysIDCommand.dynamic(SysIdRoutine.Direction.kReverse)
//    )

/** Shooting from anywhere */
    mechanismController.b().onTrue(
      ParallelCommandGroup(
        orbitCmd,
        robot.shooter.shootAnywhere(),
        robot.pivot.pivotShootAnywhere(),
        SequentialCommandGroup(
          WaitUntilCommand {
            orbitCmd.atSetpoint() &&
              robot.shooter.atSetpoint() &&
              robot.pivot.atSetpoint()
          },
          PrintCommand("GOING TO SHOOT!!!"),
          robot.undertaker.intake(),
          robot.feeder.intake()
        )
      )
    )
  }

  private fun intakePiece(): Command {
    return SequentialCommandGroup(
      robot.undertaker.intake(),
      robot.feeder.intake(),
      WaitUntilCommand { !robot.infrared.get() }
    )
  }

  private fun checkNoteInLocation(): Command {
    return ConditionalCommand(
      InstantCommand(),
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition()
      )
    ) { !robot.infrared.get() && robot.closeToShooterInfrared.get() }
      .withTimeout(FeederConstants.CHECK_NOTE_IN_LOCATION_TIMEOUT_SECONDS)
  }

  private fun slowIntake(): Command {
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

  private fun outtakeToNotePosition(): Command {
    val cmd = ConditionalCommand(
      SequentialCommandGroup(
        robot.undertaker.stop(),
        robot.feeder.outtake(),
        WaitUntilCommand { robot.closeToShooterInfrared.get() },
        stopAll()
      ),
      stopAll()
    ) { !robot.closeToShooterInfrared.get() }

    cmd.name = "outtake to note pos"
    return cmd
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
