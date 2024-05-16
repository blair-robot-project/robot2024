package frc.team449.robot2024.subsystems.shooter

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.CoastOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterKrakenConstants
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

open class SpinShooterKraken(
  val rightMotor: TalonFX,
  val leftMotor: TalonFX,
  private val robot: Robot
) : SubsystemBase() {

  private var robotAngleGoal = 0.0

  /** Left, Right Desired velocity */
  private var desiredVels = Pair(0.0, 0.0)

  open val rightVelocity: Supplier<Double> = rightMotor.velocity.asSupplier()
  open val leftVelocity: Supplier<Double> = leftMotor.velocity.asSupplier()

  private val velocityRequest = VelocityVoltage(0.0).withSlot(0).withEnableFOC(false).withUpdateFreqHz(500.0)

  private val leftRateLimiter = SlewRateLimiter(SpinShooterKrakenConstants.BRAKE_RATE_LIMIT)
  private val rightRateLimiter = SlewRateLimiter(SpinShooterKrakenConstants.BRAKE_RATE_LIMIT)

  init {
    name = "Kraken Spin Shooter"

    leftMotor.optimizeBusUtilization()
    rightMotor.optimizeBusUtilization()
    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()

    this.defaultCommand = coast()
  }

  fun hold(): Command {
    return this.run {
      shootPiece(
        desiredVels.first,
        desiredVels.second
      )
    }
  }

  fun setVoltage(volts: Double) {
    setLeftVoltage(volts)
    setRightVoltage(volts)
  }

  fun setLeftVoltage(volts: Double) {
    leftMotor.setVoltage(volts)
  }

  fun setRightVoltage(volts: Double) {
    rightMotor.setVoltage(volts)
  }

  fun shootSubwoofer(): Command {
    val cmd = this.run {
      shootPiece(
        SpinShooterKrakenConstants.SUBWOOFER_LEFT_SPEED,
        SpinShooterKrakenConstants.SUBWOOFER_RIGHT_SPEED
      )
    }
    cmd.name = "shooting subwoofer"
    return cmd
  }

  fun shootAnywhere(): Command {
    val cmd = this.run {
      shootPiece(
        SpinShooterKrakenConstants.ANYWHERE_LEFT_SPEED,
        SpinShooterKrakenConstants.ANYWHERE_RIGHT_SPEED
      )
    }
    cmd.name = "shooting anywhere speed"
    return cmd
  }

  fun shootPass(): Command {
    val cmd = this.run {
      shootPiece(
        SpinShooterKrakenConstants.PASS_LEFT_SPEED,
        SpinShooterKrakenConstants.PASS_RIGHT_SPEED
      )
    }
    cmd.name = "shooting pass speed"
    return cmd
  }

  fun podiumShot(): Command {
    val cmd = FunctionalCommand(
      { },
      {
        shootPiece(
          SpinShooterKrakenConstants.ANYWHERE_LEFT_SPEED,
          SpinShooterKrakenConstants.ANYWHERE_RIGHT_SPEED
        )

        val distance = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(FieldConstants.fieldLength - 14.144, 4.211))))
        } else {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(14.144, 4.211))))
        }

        val angle = Units.degreesToRadians(SpinShooterKrakenConstants.equation(distance) + 0.385)

        robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

        val fieldToRobot = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Translation2d(FieldConstants.fieldLength - 14.144, 4.211)
        } else {
          Translation2d(14.144, 4.211)
        }

        val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot

        robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        if (robot.shooter.atSetpoint() &&
          abs(RobotConstants.ORTHOGONAL_CONTROLLER.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
          robot.pivot.inTolerance() &&
          robot.mechController.hid.leftBumper
        ) {
          robot.feeder.intakeVoltage()
        }
      },
      { },
      { !robot.driveController.hid.leftBumper },
      this,
      robot.pivot,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "auto aiming podium"
    return cmd
  }

  fun autoLineShot(): Command {
    val cmd = FunctionalCommand(
      { },
      {
        shootPiece(
          SpinShooterKrakenConstants.ANYWHERE_LEFT_SPEED,
          SpinShooterKrakenConstants.ANYWHERE_RIGHT_SPEED
        )

        val distance = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(2.49, Units.inchesToMeters(218.42)))))
        } else {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(FieldConstants.fieldLength - 2.49, Units.inchesToMeters(218.42)))))
        }

        val angle = Units.degreesToRadians(SpinShooterKrakenConstants.equation(distance))

        robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

        robot.driveCommand.snapToAngle(0.0)

        if (robot.shooter.atSetpoint() &&
          robot.driveCommand.atGoal &&
          robot.pivot.inTolerance() &&
          robot.mechController.hid.leftBumper
        ) {
          robot.feeder.intakeVoltage()
          robot.undertaker.intakeVoltage()
        }
      },
      { },
      { !robot.driveController.hid.bButton },
      this,
      robot.pivot,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "auto aiming auto line"
    return cmd
  }

  fun passShot(): Command {
    val cmd = FunctionalCommand(
      { },
      {
        robot.feeder.stopVoltage()

        robot.pivot.moveToAngleSlow(PivotConstants.PASS_ANGLE)

        shootPiece(
          SpinShooterKrakenConstants.PASS_LEFT_SPEED,
          SpinShooterKrakenConstants.PASS_RIGHT_SPEED
        )

        val fieldToRobot = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Translation2d(FieldConstants.fieldLength - 7.5, 1.585)
        } else {
          Translation2d(7.5, 1.585)
        }

        val robotToPoint = FieldConstants.PASS_POSE - fieldToRobot

        robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        if (robot.shooter.atAimSetpoint() &&
          abs(RobotConstants.ORTHOGONAL_CONTROLLER.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
          robot.mechController.hid.leftBumper &&
          robot.pivot.inTolerance()
        ) {
          robot.feeder.intakeVoltage()
          robot.undertaker.intakeVoltage()
        }
      },
      { },
      { !robot.driveController.hid.xButton },
      this,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "pass shot"
    return cmd
  }

  fun passShotT2(): Command {
    val cmd = FunctionalCommand(
      { },
      {
        robot.feeder.stopVoltage()

        robot.pivot.moveToAngleSlow(PivotConstants.PASS_ANGLE_T2)

        shootPiece(
          SpinShooterKrakenConstants.PASS2_LEFT_SPEED,
          SpinShooterKrakenConstants.PASS2_RIGHT_SPEED
        )

        val fieldToRobot = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Translation2d(FieldConstants.fieldLength - 6.761, 4.259)
        } else {
          Translation2d(6.761, 4.259)
        }

        val robotToPoint = FieldConstants.PASS_POSE - fieldToRobot

        robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        if (robot.shooter.atAimSetpoint() &&
          abs(RobotConstants.ORTHOGONAL_CONTROLLER.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
          robot.mechController.hid.leftBumper &&
          robot.pivot.inTolerance()
        ) {
          robot.feeder.intakeVoltage()
          robot.undertaker.intakeVoltage()
        }
      },
      { },
      { !robot.driveController.hid.bButton },
      this,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "pass shot"
    return cmd
  }

  fun passShotT3(): Command {
    val cmd = FunctionalCommand(
      { },
      {
        robot.feeder.stopVoltage()

        robot.pivot.moveToAngleSlow(PivotConstants.PASS_ANGLE_T3)

        shootPiece(
          SpinShooterKrakenConstants.PASS3_LEFT_SPEED,
          SpinShooterKrakenConstants.PASS3_RIGHT_SPEED
        )

        val fieldToRobot = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Translation2d(FieldConstants.fieldLength - 7.053, 6.846)
        } else {
          Translation2d(7.053, 6.846)
        }

        val robotToPoint = FieldConstants.PASS_POSE - fieldToRobot

        robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        if (robot.shooter.atAimSetpoint() &&
          abs(RobotConstants.ORTHOGONAL_CONTROLLER.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
          robot.mechController.hid.leftBumper &&
          robot.pivot.inTolerance()
        ) {
          robot.feeder.intakeVoltage()
          robot.undertaker.intakeVoltage()
        }
      },
      { },
      { !robot.driveController.hid.yButton },
      this,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "pass shot"
    return cmd
  }

  fun autoAim(): Command {
    val cmd = FunctionalCommand(
      {
        val robotToPoint = FieldConstants.SPEAKER_POSE - robot.drive.pose.translation
        robotAngleGoal = (robotToPoint.angle + Rotation2d(if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)).radians
      },
      {
        robot.feeder.stopVoltage()

        shootPiece(
          SpinShooterKrakenConstants.ANYWHERE_LEFT_SPEED,
          SpinShooterKrakenConstants.ANYWHERE_RIGHT_SPEED
        )

        val fieldRelSpeed = ChassisSpeeds(
          robot.drive.currentSpeeds.vxMetersPerSecond * robot.drive.heading.cos - robot.drive.currentSpeeds.vyMetersPerSecond * robot.drive.heading.sin,
          robot.drive.currentSpeeds.vyMetersPerSecond * robot.drive.heading.cos + robot.drive.currentSpeeds.vxMetersPerSecond * robot.drive.heading.sin,
          robot.drive.currentSpeeds.omegaRadiansPerSecond
        )

        var noComp = false

        if (hypot(fieldRelSpeed.vxMetersPerSecond, fieldRelSpeed.vyMetersPerSecond) < 0.15) {
          noComp = true
        }

        var virtualTarget = FieldConstants.SPEAKER_POSE

        for (i in 0..5) {
          if (noComp) {
            break
          }

          val distance = abs(virtualTarget.getDistance(robot.drive.pose.translation))

          val shotTime = SpinShooterKrakenConstants.TIME_MAP.get(distance)

          virtualTarget =
            Translation2d(
              FieldConstants.SPEAKER_POSE.x - shotTime * fieldRelSpeed.vxMetersPerSecond,
              FieldConstants.SPEAKER_POSE.y - shotTime * fieldRelSpeed.vyMetersPerSecond,
            )

          val newShotTime = SpinShooterKrakenConstants.TIME_MAP.get(distance)

          if (shotTime - newShotTime < 1e-3) {
            break
          }
        }

        val distance = abs(virtualTarget.getDistance(robot.drive.pose.translation))

        val angle = if (distance <= 1.30) 0.0 else SpinShooterKrakenConstants.SHOOTING_MAP.get(distance)

        robot.pivot.moveToAngleAuto(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

        val robotToPoint = virtualTarget - robot.drive.pose.translation

        val desiredAngle = robotToPoint.angle + Rotation2d(if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        robot.driveCommand.snapToAngle(desiredAngle.radians, 0.0)

//        println(RobotConstants.ORTHOGONAL_CONTROLLER.positionError * 12.5)
//        println(Units.radiansToDegrees(RobotConstants.ORTHOGONAL_CONTROLLER.positionError))

        if (robot.shooter.atAimSetpoint() &&
          abs(RobotConstants.ORTHOGONAL_CONTROLLER.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
          robot.pivot.inShootAnywhereTolerance(angle) &&
          robot.mechController.hid.leftBumper
        ) {
          robot.feeder.intakeVoltage()
        }
      },
      { },
      {
        !robot.driveController.hid.leftBumper
      },
      this,
      robot.pivot,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "auto aiming"
    return cmd
  }

  fun atSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterKrakenConstants.IN_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterKrakenConstants.IN_TOLERANCE
  }

  fun atAimSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterKrakenConstants.AIM_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterKrakenConstants.AIM_TOLERANCE
  }

  fun atAutoSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterKrakenConstants.AUTO_SHOOT_TOL &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterKrakenConstants.AUTO_SHOOT_TOL &&
      desiredVels.first != 0.0 &&
      desiredVels.second != 0.0
  }

  fun atAmpSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterKrakenConstants.AMP_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterKrakenConstants.AMP_TOLERANCE
  }

  fun scoreAmp(): Command {
    val cmd = this.run {
      shootPiece(SpinShooterKrakenConstants.AMP_SPEED, SpinShooterKrakenConstants.AMP_SPEED)
    }
    cmd.name = "scoring amp"
    return cmd
  }

  fun duringIntake(): Command {
    val cmd = this.run {
      shootPiece(SpinShooterKrakenConstants.OUTTAKE_SPEED, SpinShooterKrakenConstants.OUTTAKE_SPEED)
    }
    cmd.name = "during intake"
    return cmd
  }

  fun shootPiece(leftSpeed: Double, rightSpeed: Double) {
    desiredVels = Pair(leftSpeed, rightSpeed)
    rightMotor.setControl(velocityRequest.withVelocity(rightSpeed))
    leftMotor.setControl(velocityRequest.withVelocity(leftSpeed))
  }

  fun coast(): Command {
    val cmd = this.runOnce {
      leftMotor.setControl(CoastOut())
      rightMotor.setControl(CoastOut())
    }
    cmd.name = "coasting shooter"
    return cmd
  }

  fun forceStop(): Command {
    val cmd = this.run {
      shootPiece(0.0, 0.0)
    }
    cmd.name = "force stop"
    return ParallelDeadlineGroup(
      WaitUntilCommand {
        abs(leftVelocity.get() - desiredVels.first) < SpinShooterKrakenConstants.MIN_COAST_VEL &&
          abs(rightVelocity.get() - desiredVels.second) < SpinShooterKrakenConstants.MIN_COAST_VEL
      },
      cmd
    ).andThen(
      coast()
    )
  }

  fun rampStop(): Command {
    val cmd = SequentialCommandGroup(
      this.runOnce {
        leftRateLimiter.reset(leftVelocity.get())
        rightRateLimiter.reset(rightVelocity.get())
      },
      this.run {
        shootPiece(
          leftRateLimiter.calculate(0.0),
          rightRateLimiter.calculate(0.0)
        )
      }.until {
        abs(leftVelocity.get()) < SpinShooterKrakenConstants.MIN_COAST_VEL &&
          abs(rightVelocity.get()) < SpinShooterKrakenConstants.MIN_COAST_VEL
      }.andThen(
        coast()
      )
    )
    cmd.name = "active stop"
    return cmd
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Right Voltage", { rightMotor.motorVoltage.value }, null)
    builder.addDoubleProperty("1.2 Last Left Voltage", { leftMotor.motorVoltage.value }, null)
    builder.publishConstString("2.0", "Current and Desired Velocities")
    builder.addDoubleProperty("2.1 Left Current Speed", { leftVelocity.get() }, null)
    builder.addDoubleProperty("2.2 Right Current Speed", { rightVelocity.get() }, null)
    builder.addDoubleProperty("2.3 Left Desired Speed", { desiredVels.first }, null)
    builder.addDoubleProperty("2.4 Right Desired Speed", { desiredVels.second }, null)
    builder.publishConstString("3.0", "Velocity Errors")
    builder.addDoubleProperty("3.1 Left Vel Error", { leftVelocity.get() - desiredVels.first }, null)
    builder.addDoubleProperty("3.2 Right Vel Error", { rightVelocity.get() - desiredVels.second }, null)
    builder.addBooleanProperty("3.3 In tolerance", ::atAimSetpoint, null)
    builder.publishConstString("4.0", "Encoder Positions")
    builder.addDoubleProperty("4.1 Left Enc Pos", { leftMotor.position.value }, null)
    builder.addDoubleProperty("4.2 Left Enc Pos", { rightMotor.position.value }, null)
    builder.publishConstString("6.0", "Shoot from Anywhere")
    builder.addDoubleProperty("6.1 Speaker Distance (meters)", { abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation)) }, null)
    builder.addBooleanProperty("6.2 Drive In Angle Tol", { abs(RobotConstants.ORTHOGONAL_CONTROLLER.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD }, null)
  }

  companion object {
    fun createKrakenSpinShooter(robot: Robot): SpinShooterKraken {
      val rightMotor = TalonFX(SpinShooterKrakenConstants.RIGHT_MOTOR_ID)
      val rightConfig = TalonFXConfiguration()
      rightConfig.CurrentLimits.StatorCurrentLimitEnable = true
      rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true
      rightConfig.CurrentLimits.StatorCurrentLimit = SpinShooterKrakenConstants.STATOR_CURRENT_LIMIT
      rightConfig.CurrentLimits.SupplyCurrentLimit = SpinShooterKrakenConstants.SUPPLY_CURRENT_LIMIT
      rightConfig.CurrentLimits.SupplyCurrentThreshold = SpinShooterKrakenConstants.BURST_CURRENT_LIMIT
      rightConfig.CurrentLimits.SupplyTimeThreshold = SpinShooterKrakenConstants.BURST_TIME_LIMIT
      rightConfig.Slot0.kS = SpinShooterKrakenConstants.RIGHT_KS
      rightConfig.Slot0.kV = SpinShooterKrakenConstants.RIGHT_KV
      rightConfig.Slot0.kA = SpinShooterKrakenConstants.RIGHT_KA
      rightConfig.Slot0.kP = SpinShooterKrakenConstants.RIGHT_KP
      rightConfig.Slot0.kI = SpinShooterKrakenConstants.RIGHT_KI
      rightConfig.Slot0.kD = SpinShooterKrakenConstants.RIGHT_KD
      rightConfig.MotorOutput.Inverted = SpinShooterKrakenConstants.RIGHT_MOTOR_ORIENTATION
      rightConfig.MotorOutput.NeutralMode = SpinShooterKrakenConstants.RIGHT_NEUTRAL_MODE
      rightConfig.MotorOutput.DutyCycleNeutralDeadband = SpinShooterKrakenConstants.DUTY_CYCLE_DEADBAND
      rightConfig.Feedback.SensorToMechanismRatio = SpinShooterKrakenConstants.GEARING
      rightMotor.configurator.apply(rightConfig)
      rightMotor.velocity.setUpdateFrequency(SpinShooterKrakenConstants.UPDATE_FREQUENCY)
      rightMotor.motorVoltage.setUpdateFrequency(SpinShooterKrakenConstants.UPDATE_FREQUENCY)
      rightMotor.closedLoopError.setUpdateFrequency(SpinShooterKrakenConstants.UPDATE_FREQUENCY)
      rightMotor.optimizeBusUtilization()

      val leftMotor = TalonFX(SpinShooterKrakenConstants.LEFT_MOTOR_ID)
      val leftConfig = TalonFXConfiguration()
      leftConfig.CurrentLimits.StatorCurrentLimitEnable = true
      leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true
      leftConfig.CurrentLimits.StatorCurrentLimit = SpinShooterKrakenConstants.STATOR_CURRENT_LIMIT
      leftConfig.CurrentLimits.SupplyCurrentLimit = SpinShooterKrakenConstants.SUPPLY_CURRENT_LIMIT
      leftConfig.CurrentLimits.SupplyCurrentThreshold = SpinShooterKrakenConstants.BURST_CURRENT_LIMIT
      leftConfig.CurrentLimits.SupplyTimeThreshold = SpinShooterKrakenConstants.BURST_TIME_LIMIT
      leftConfig.Slot0.kS = SpinShooterKrakenConstants.LEFT_KS
      leftConfig.Slot0.kV = SpinShooterKrakenConstants.LEFT_KV
      leftConfig.Slot0.kA = SpinShooterKrakenConstants.LEFT_KA
      leftConfig.Slot0.kP = SpinShooterKrakenConstants.LEFT_KP
      leftConfig.Slot0.kI = SpinShooterKrakenConstants.LEFT_KI
      leftConfig.Slot0.kD = SpinShooterKrakenConstants.LEFT_KD
      leftConfig.MotorOutput.Inverted = SpinShooterKrakenConstants.LEFT_MOTOR_ORIENTATION
      leftConfig.MotorOutput.NeutralMode = SpinShooterKrakenConstants.LEFT_NEUTRAL_MODE
      leftConfig.MotorOutput.DutyCycleNeutralDeadband = SpinShooterKrakenConstants.DUTY_CYCLE_DEADBAND
      leftConfig.Feedback.SensorToMechanismRatio = SpinShooterKrakenConstants.GEARING
      leftMotor.configurator.apply(leftConfig)
      leftMotor.velocity.setUpdateFrequency(SpinShooterKrakenConstants.UPDATE_FREQUENCY)
      leftMotor.motorVoltage.setUpdateFrequency(SpinShooterKrakenConstants.UPDATE_FREQUENCY)
      leftMotor.closedLoopError.setUpdateFrequency(SpinShooterKrakenConstants.UPDATE_FREQUENCY)
      leftMotor.optimizeBusUtilization()

      return SpinShooterKraken(
        rightMotor,
        leftMotor,
        robot
      )
    }
  }
}
