package frc.team449.robot2024.subsystems

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure.mutable
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
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

  val sysIdRoutine = SysIdRoutine(
    SysIdRoutine.Config(),
    Mechanism(
      { voltage: Measure<Voltage> -> robot.drive.setVoltage(voltage.`in`(Volts)) },
      null,
      robot.drive
    )
  )

  private val m_appliedVoltage = mutable(Volts.of(0.0))
  private val m_angle = mutable(Radians.of(0.0))
  private val m_velocity = mutable(RadiansPerSecond.of(0.0))

  val shooterRoutine = SysIdRoutine(
    SysIdRoutine.Config(
      Volts.of(0.5).per(Seconds.of(1.0)),
      Volts.of(3.0),
      Seconds.of(10.0)
    ) { state -> SignalLogger.writeString("state", state.toString()) },
    Mechanism(
      { voltage: Measure<Voltage> -> run { robot.shooter.setVoltage(voltage.`in`(Volts)) } },
      null,
      robot.shooter,
      "shooter"
    )
  )

  val pivotRoutine = SysIdRoutine(
    SysIdRoutine.Config(
      Volts.of(0.085).per(Seconds.of(1.0)),
      Volts.of(0.75),
      Seconds.of(17.5)
    ),
    Mechanism(
      { voltage: Measure<Voltage> ->
        run {
          robot.pivot.setVoltage(voltage.`in`(Volts))
        }
      },
      null,
      robot.pivot,
      "pivot"
    )
  )

  val orbitCmd = OrbitAlign(
    robot.drive,
    robot.driveController,
    { FieldConstants.SPEAKER_POSE }
  )

  private fun stopAll(): Command {
    return ParallelCommandGroup(
      robot.undertaker.stop(),
      robot.shooter.rampStop(),
      robot.feeder.stop()
    )
  }

  private fun stopIntake(): Command {
    return ParallelCommandGroup(
      robot.undertaker.stop(),
      robot.feeder.stop()
    )
  }

  private fun robotBindings() {
    Trigger { robot.infrared.get() }.onFalse(
      SequentialCommandGroup(
        InstantCommand({
          robot.driveController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.1)
          robot.mechController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.3)
        }),
        WaitCommand(0.5804),
        InstantCommand({
          robot.driveController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
          robot.mechController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        }),
      )
    )

    mechanismController.y().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
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

//    mechanismController.povLeft().onTrue(
//      SequentialCommandGroup(
//        checkNoteInLocation(),
//        ParallelCommandGroup(
//          robot.undertaker.slowIntake().andThen(
//            WaitCommand(0.25),
//          robot.pivot.movePass(),
//          ),
//          robot.shooter.shootPass()
//        )
//      )
//    )

    robot.mechController.povRight().onTrue(
      robot.pivot.moveAmp().alongWith(
        SequentialCommandGroup(
          WaitUntilCommand { robot.pivot.inAmpTolerance() },
          robot.climber.extend().withTimeout(1.55),
          robot.climber.stop()
        )
      )
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

    driveController.leftBumper().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        robot.undertaker.slowIntake(),
        WaitCommand(0.10),
        robot.shooter.podiumShot()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.undertaker.stop(),
        robot.shooter.rampStop(),
        robot.feeder.stop(),
        robot.pivot.moveStow()
      )
    )

    driveController.x().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        robot.shooter.passShotSourceSide()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.undertaker.stop(),
        robot.shooter.rampStop(),
        robot.feeder.stop(),
        robot.pivot.moveStow()
      )
    )

    driveController.b().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        robot.shooter.passShotBehindStage()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.undertaker.stop(),
        robot.shooter.rampStop(),
        robot.feeder.stop(),
        robot.pivot.moveStow()
      )
    )

    driveController.y().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        robot.shooter.passShotAmpSide()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.undertaker.stop(),
        robot.shooter.rampStop(),
        robot.feeder.stop(),
        robot.pivot.moveStow()
      )
    )

    mechanismController.leftTrigger().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        ParallelCommandGroup(
          robot.undertaker.slowIntake().andThen(
            WaitUntilCommand { robot.shooter.atAmpSetpoint() && robot.pivot.inAmpTolerance() },
            robot.feeder.intake()
          ),
          SequentialCommandGroup(
            WaitCommand(0.125),
            robot.pivot.moveAmp()
          ),
          robot.shooter.scoreAmp()
        )
      )
        .until { robot.infrared.get() && robot.closeToShooterInfrared.get() }
        .andThen(
          WaitCommand(0.15),
          stopAll()
            .alongWith(robot.pivot.moveStow())
        )
    ) // .onFalse(
//      robot.feeder.intake()
//        .alongWith(
//          robot.shooter.scoreAmp(),
//          robot.pivot.moveAmp()
//        )
//        .until { robot.infrared.get() && robot.closeToShooterInfrared.get() }
//        .unless { !robot.infrared.get() || !robot.closeToShooterInfrared.get() }
//        .andThen(
//          stopAll()
//            .alongWith(robot.pivot.moveStow())
//        )
//    )

    driveController.leftTrigger().onTrue(
      ParallelCommandGroup(
        robot.feeder.outtake(),
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
        SequentialCommandGroup(
          slowIntake(),
          outtakeToNotePosition()
        )
          .repeatedly()
          .withTimeout(FeederConstants.CHECK_NOTE_IN_LOCATION_TIMEOUT_SECONDS)
          .andThen(checkNoteInLocation())
      )
    ) // .onFalse(
//      SequentialCommandGroup(
//        slowIntake(),
//        outtakeToNotePosition()
//      )
//        .repeatedly()
//        .withTimeout(FeederConstants.CHECK_NOTE_IN_LOCATION_TIMEOUT_SECONDS)
//        .andThen(checkNoteInLocation())
//    )

//    mechanismController.x().onTrue(
//      SequentialCommandGroup(
//        checkNoteInLocation(),
//        WaitUntilCommand { robot.shooter.atSetpoint() },
//        robot.feeder.intake(),
//        robot.undertaker.intake(),
//      ).alongWith(
//        robot.shooter.shootPass(),
//        robot.pivot.movePass()
//      )
//    ).onFalse(
//      stopAll().alongWith(
//        robot.pivot.moveStow()
//      )
//    )

    mechanismController.back().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition()
      )
    )

    mechanismController.start().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition(),
        checkNoteInLocation(),
        ParallelCommandGroup(
          robot.shooter.shootAnywhere(),
        )
      )
    )

    mechanismController.rightBumper().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition(),
        checkNoteInLocation(),
        ParallelCommandGroup(
          robot.shooter.shootSubwoofer(),
          robot.pivot.moveStow()
        )
      )
    )

    mechanismController.rightTrigger().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
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
    // Quasistatic Forwards
//    driveController.povUp().onTrue(
//      shooterRoutine.quasistatic(SysIdRoutine.Direction.kForward)
//    )
//
//    // Quasistatic Reverse
//    driveController.povDown().onTrue(
//      shooterRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
//    )
//
//    // Dynamic Forwards
//    driveController.povRight().onTrue(
//      shooterRoutine.dynamic(SysIdRoutine.Direction.kForward)
//    )
//
//    // Dynamic Reverse
//    driveController.povLeft().onTrue(
//      shooterRoutine.dynamic(SysIdRoutine.Direction.kReverse)
//    )
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
      stopIntake(),
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
        stopIntake()
      ),
      stopIntake()
    ) { !robot.closeToShooterInfrared.get() }

    cmd.name = "outtake to note pos"
    return cmd
  }

  private fun nonRobotBindings() {
    // slow drive
    driveController.rightBumper().onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 }))
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
//    driveController.b().onTrue(
//      ConditionalCommand(
//        InstantCommand({
//          robot.drive as SwerveSim
//          robot.drive.resetPos()
//        }),
//        InstantCommand()
//      ) { RobotBase.isSimulation() }
//    )
  }

  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
