package frc.team449.robot2024.subsystems

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.control.auto.PIDPoseAlign
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.auto.AutoConstants
import frc.team449.robot2024.constants.drives.SwerveConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.FeederConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

class NewControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  private fun robotBindings() {
    rumbleControllers()

    premoveAmp()
    scoreAmp()

//    manualClimb()
    climbersDown()
    autoClimbExtend()

    manualPivot()
    stowPivot()

    subwooferShot()
    /** Note: If you change this binding, change the isFinished portion of the autoAim command  in the shooter subsystem as well
     * Also there is no mech confirmation anymore */
    visionShot()
//    podiumShot()

    sourcePass()
    stagePass()
//    ampPass()

    outtake()
    intake()
    manualJiggle()

    spinupFarShot()
    spinupSubwoofer()

    forceStop()

    /** In order to use this, click mechanism controller left d-pad
     * and then under the pivot section in networktables,
     * adjust the Calibration Angle in degrees  (copy over the radian value in code though).
     * You also want to check that the pivot is in AUTO tolerance not just normal tolerance.
     *
     * For physical setup, I'd recommend actually intaking through the intake for every shot (but center the note as good as you can by hand) */
//    calibrateAutoAimMap()
  }

  private fun nonRobotBindings() {
    // slowDrive()

    resetGyro()

    // addNoiseToSimulatedPose()
  }

  private fun slowDrive() {
    driveController.rightBumper().onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(
          InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
        )
    )
  }

  private fun resetGyro() {
    driveController.start().onTrue(
      ConditionalCommand(
        InstantCommand({ robot.drive.heading = Rotation2d(PI) }),
        InstantCommand({ robot.drive.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
    )
  }

  private fun addNoiseToSimulatedPose() {
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

  private fun forceStop() {
    mechanismController.b().onTrue(
      stopAll()
    )
  }

  private fun rumbleControllers() {
    Trigger { robot.infrared.get() }.onFalse(
      SequentialCommandGroup(
        InstantCommand({
          robot.driveController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.1)
          robot.mechController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.3)
        }),
        WaitCommand(0.5804),
        InstantCommand({
          robot.driveController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        }),
        WaitCommand(1.0 - 0.5804),
        InstantCommand({
          robot.mechController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        })
      )
    )
  }

  private fun premoveAmp() {
    mechanismController.leftTrigger().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        ParallelCommandGroup(
          robot.undertaker.slowIntake(),
          SequentialCommandGroup(
            WaitCommand(0.10),
            robot.pivot.moveAmp()
          ),
          robot.shooter.scoreAmp()
        )
      )
    )
  }

  private fun manualClimb() {
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
  }

  private fun autoClimbExtend() {
    robot.driveController.a().onTrue(
      robot.pivot.moveAmp().alongWith(
        SequentialCommandGroup(
          WaitUntilCommand { robot.pivot.inAmpTolerance() },
          robot.climber.extend().withTimeout(1.55),
          robot.climber.stop()
        )
      )
    )
  }

  private fun climbersDown() {
    robot.driveController.b().onTrue(
      robot.climber.retract()
    ).onFalse(
      robot.climber.stop()
    )
  }

  private fun manualPivot() {
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
  }

  private fun stowPivot() {
    mechanismController.a().onTrue(
      robot.pivot.moveStow().alongWith(
        stopAll()
      )
    )
  }

  private fun podiumShot() {
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
  }

  private fun sourcePass() {
    mechanismController.x().onTrue(
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
  }

  private fun stagePass() {
    mechanismController.y().onTrue(
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
  }

  private fun ampPass() {
    driveController.b().onTrue(
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
  }

  private fun scoreAmp() {
    driveController.rightBumper().onTrue(
      ConditionalCommand(
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
            WaitCommand(0.1),
            stopAll()
              .alongWith(robot.pivot.moveStow())
          ),
        SequentialCommandGroup(
          checkNoteInLocation(),
          ParallelCommandGroup(
            robot.undertaker.slowIntake().andThen(
              WaitUntilCommand { robot.shooter.atAmpSetpoint() && robot.pivot.inAmpTolerance() },
              robot.feeder.intake()
            ),
            robot.pivot.moveAmp(),
            robot.shooter.scoreAmp()
          )
        )
          .until { robot.infrared.get() && robot.closeToShooterInfrared.get() }
          .andThen(
            WaitCommand(0.1),
            stopAll()
              .alongWith(robot.pivot.moveStow())
          )
      ) { robot.pivot.inStowTolerance() }
    )
  }

  private fun outtake() {
    mechanismController.rightTrigger().onTrue(
      ParallelCommandGroup(
        robot.feeder.outtake(),
        robot.undertaker.outtake()
      )
    ).onFalse(
      stopAll()
    )
  }

  private fun intake() {
    driveController.leftTrigger().onTrue(
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
    )
  }

  private fun manualJiggle() {
    mechanismController.back().onTrue(
      SequentialCommandGroup(
        slowIntake(),
        outtakeToNotePosition()
      )
    )
  }

  private fun spinupFarShot() {
    mechanismController.start().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        ParallelCommandGroup(
          robot.shooter.shootAnywhere(),
        )
      )
    )
  }

  private fun spinupSubwoofer() {
    mechanismController.rightBumper().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        ParallelCommandGroup(
          robot.shooter.shootSubwoofer(),
          robot.pivot.moveStow()
        )
      )
    )
  }

  private fun subwooferShot() {
    driveController.rightTrigger().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        WaitUntilCommand { robot.shooter.atSetpoint() },
        robot.feeder.intake(),
        robot.undertaker.intake()
      )
        .alongWith(robot.shooter.shootSubwoofer())
        .until { robot.infrared.get() && robot.closeToShooterInfrared.get() }
        .andThen(
          WaitCommand(0.150),
          stopAll()
        )
    ).onFalse(
      stopAll()
    )
  }

  private fun intakePiece(): Command {
    return SequentialCommandGroup(
      robot.undertaker.intake(),
      robot.feeder.intake(),
      WaitUntilCommand { !robot.infrared.get() }
    )
  }

  private fun calibrateAutoAimMap() {
    driveController.leftBumper().onTrue(
      SequentialCommandGroup(
        checkNoteInLocation(),
        verySlowIntake(),
        robot.undertaker.slowIntake(),
        WaitCommand(0.050),
        robot.shooter.autoAim(calibrationMode = true)
      )
    )
  }

  private fun autoAmpAlign() {
    driveController.a().onTrue(
      ConditionalCommand(
        FollowPathHolonomic(
          PathPlannerPath(
            PathPlannerPath.bezierFromPoses(
              Pose2d(1.825, 6.0, Rotation2d.fromDegrees(90.0)),
              Pose2d(1.825, 7.25, Rotation2d.fromDegrees(90.0)),
              Pose2d(1.825, 7.75, Rotation2d.fromDegrees(90.0))
            ),
            PathConstraints(4.0, 4.0, 2 * PI, 8 * PI),
            GoalEndState(0.0, Rotation2d.fromDegrees(-90.0))
          ),
          robot.drive::pose,
          robot.drive::currentSpeeds,
          robot.drive::set,
          HolonomicPathFollowerConfig(
            PIDConstants(AutoConstants.DEFAULT_X_KP, AutoConstants.DEFAULT_X_KD, 0.0),
            PIDConstants(AutoConstants.DEFAULT_ROTATION_KP, AutoConstants.DEFAULT_ROTATION_KD, 0.0),
            SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED,
            0.3486433367353,
            ReplanningConfig(
              true,
              true,
              1.0,
              0.20
            )
          ),
          { false },
          robot.drive
        ).andThen(
          PIDPoseAlign(
            robot.drive,
            Pose2d(1.825, 7.75, Rotation2d.fromDegrees(-90.0)),
            timeout = 0.85
          )
        ),
        FollowPathHolonomic(
          PathPlannerPath(
            PathPlannerPath.bezierFromPoses(
              Pose2d(FieldConstants.fieldLength - 1.825, 6.0, Rotation2d.fromDegrees(90.0)),
              Pose2d(FieldConstants.fieldLength - 1.825, 7.25, Rotation2d.fromDegrees(90.0)),
              Pose2d(FieldConstants.fieldLength - 1.825, 7.75, Rotation2d.fromDegrees(90.0))
            ),
            PathConstraints(4.0, 4.0, 2 * PI, 8 * PI),
            GoalEndState(0.0, Rotation2d.fromDegrees(-90.0))
          ),
          robot.drive::pose,
          robot.drive::currentSpeeds,
          robot.drive::set,
          HolonomicPathFollowerConfig(
            PIDConstants(AutoConstants.DEFAULT_X_KP, AutoConstants.DEFAULT_X_KD, 0.0),
            PIDConstants(AutoConstants.DEFAULT_ROTATION_KP, AutoConstants.DEFAULT_ROTATION_KD, 0.0),
            SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED,
            0.3486433367353,
            ReplanningConfig(
              true,
              true,
              1.0,
              0.20
            )
          ),
          { false },
          robot.drive
        ).andThen(
          PIDPoseAlign(
            robot.drive,
            Pose2d(FieldConstants.fieldLength - 1.825, 7.75, Rotation2d.fromDegrees(-90.0)),
            timeout = 0.85
          )
        )
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue }
        .deadlineWith(
          robot.undertaker.intake()
            .andThen(
              WaitCommand(0.175),
              robot.pivot.moveAmp()
                .alongWith(
                  robot.shooter.scoreAmp()
                )
            )
        )
        .andThen(
          robot.pivot.moveAmp()
            .alongWith(
              robot.shooter.scoreAmp(),
              WaitUntilCommand { robot.pivot.inAmpTolerance() && robot.shooter.atAmpSetpoint() }
                .andThen(robot.feeder.intake())
            )
            .until { robot.infrared.get() && robot.closeToShooterInfrared.get() }
        )
    )
  }

  private fun visionShot() {
    driveController.leftBumper().onTrue(
      SequentialCommandGroup(
        InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
          .andThen(InstantCommand({ robot.drive.maxRotSpeed = 0.0 })),
//        checkNoteInLocation(),
//        verySlowIntake(),
        robot.undertaker.slowIntake(),
        WaitCommand(0.050),
        robot.shooter.autoAim()
      )
    ).onFalse(
      ParallelCommandGroup(
        InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
          .andThen(InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })),
        robot.undertaker.stop(),
        robot.shooter.rampStop(),
        robot.feeder.stop(),
        robot.pivot.moveStow()
      )
    )
  }

  /** Characterization functions */
  private fun shooterCharacterization() {
    val shooterRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds.of(1.0)),
        Volts.of(3.0),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      Mechanism(
        { voltage: Measure<Voltage> ->
          run { robot.shooter.setVoltage(voltage.`in`(Volts)) }
        },
        null,
        robot.shooter,
        "shooter"
      )
    )

    // Quasistatic Forwards
    driveController.povUp().onTrue(
      shooterRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    driveController.povDown().onTrue(
      shooterRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    driveController.povRight().onTrue(
      shooterRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    driveController.povLeft().onTrue(
      shooterRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  private fun pivotCharacterization() {
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

    // Quasistatic Forwards
    driveController.povUp().onTrue(
      pivotRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    driveController.povDown().onTrue(
      pivotRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    driveController.povRight().onTrue(
      pivotRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    driveController.povLeft().onTrue(
      pivotRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  private fun driveCharacterization() {
    val driveRoutine = SysIdRoutine(
      SysIdRoutine.Config(),
      Mechanism(
        { voltage: Measure<Voltage> -> robot.drive.setVoltage(voltage.`in`(Volts)) },
        null,
        robot.drive
      )
    )

    // Quasistatic Forwards
    driveController.povUp().onTrue(
      driveRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    driveController.povDown().onTrue(
      driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    driveController.povRight().onTrue(
      driveRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    driveController.povLeft().onTrue(
      driveRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  /** Misc helper functions */
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

  private fun verySlowIntake(): Command {
    return ConditionalCommand(
      SequentialCommandGroup(
        ParallelCommandGroup(
          robot.undertaker.slowIntake(),
          robot.feeder.verySlowIntake(),
        ),
        WaitUntilCommand { !robot.closeToShooterInfrared.get() },
        robot.feeder.stop()
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

  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
