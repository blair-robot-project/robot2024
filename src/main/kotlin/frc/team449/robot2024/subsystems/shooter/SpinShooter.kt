package frc.team449.robot2024.subsystems.shooter

import edu.wpi.first.math.*
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import java.util.function.Supplier
import kotlin.Pair
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

open class SpinShooter(
  val rightMotor: WrappedMotor,
  val leftMotor: WrappedMotor,
  private val leftController: LinearQuadraticRegulator<N1, N1, N1>,
  private val rightController: LinearQuadraticRegulator<N1, N1, N1>,
  private val leftObserver: KalmanFilter<N2, N1, N1>,
  private val rightObserver: KalmanFilter<N2, N1, N1>,
  private val leftFeedforward: LinearPlantInversionFeedforward<N1, N1, N1>,
  private val rightFeedforward: LinearPlantInversionFeedforward<N1, N1, N1>,
  private val robot: Robot
) : SubsystemBase() {

  /** Left, Right Desired velocity */
  private var desiredVels = Pair(0.0, 0.0)

  open val rightVelocity: Supplier<Double> =
    Supplier { rightMotor.velocity }

  open val leftVelocity: Supplier<Double> =
    Supplier { leftMotor.velocity }

  private val leftRateLimiter = SlewRateLimiter(SpinShooterConstants.BRAKE_RATE_LIMIT)
  private val rightRateLimiter = SlewRateLimiter(SpinShooterConstants.BRAKE_RATE_LIMIT)

  init {
    leftController.reset()

    leftFeedforward.reset(
      VecBuilder.fill(
        desiredVels.first
      )
    )

    leftObserver.xhat = VecBuilder.fill(
      desiredVels.first,
      0.0
    )

    rightController.reset()

    rightFeedforward.reset(
      VecBuilder.fill(
        desiredVels.first
      )
    )

    rightObserver.xhat = VecBuilder.fill(
      desiredVels.first,
      0.0
    )

    this.defaultCommand = coast()
  }

  private fun correct() {
    val voltages = getVoltages()

    leftObserver.correct(
      VecBuilder.fill(voltages.first),
      VecBuilder.fill(leftVelocity.get())
    )

    rightObserver.correct(
      VecBuilder.fill(voltages.second),
      VecBuilder.fill(rightVelocity.get())
    )
  }

  private fun predict() {
    val leftVoltage = leftController.calculate(
      VecBuilder.fill(
        leftObserver.getXhat(0)
      ),
      VecBuilder.fill(
        desiredVels.first
      )
    ).plus(
      leftFeedforward.calculate(
        VecBuilder.fill(
          desiredVels.first
        )
      )
    ).plus(
      -leftObserver.getXhat(1)
    )

    leftObserver.predict(leftVoltage, RobotConstants.LOOP_TIME)

    val rightVoltage = rightController.calculate(
      VecBuilder.fill(
        rightObserver.getXhat(0)
      ),
      VecBuilder.fill(
        desiredVels.second
      )
    ).plus(
      rightFeedforward.calculate(
        VecBuilder.fill(
          desiredVels.second
        )
      )
    ).plus(
      -rightObserver.getXhat(1)
    )

    rightObserver.predict(rightVoltage, RobotConstants.LOOP_TIME)
  }

  private fun getVoltages(): Pair<Double, Double> {
    val leftVoltage = MathUtil.clamp(
      leftController.getU(0) +
        leftFeedforward.getUff(0) -
        leftObserver.getXhat(1) +
        sign(desiredVels.first) * SpinShooterConstants.LEFT_KS,
      -SpinShooterConstants.MAX_VOLTAGE,
      SpinShooterConstants.MAX_VOLTAGE
    )

    val rightVoltage = MathUtil.clamp(
      rightController.getU(0) +
        rightFeedforward.getUff(0) -
        rightObserver.getXhat(1) +
        sign(desiredVels.second) * SpinShooterConstants.RIGHT_KS,
      -SpinShooterConstants.MAX_VOLTAGE,
      SpinShooterConstants.MAX_VOLTAGE
    )

    return Pair(leftVoltage, rightVoltage)
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
        SpinShooterConstants.SUBWOOFER_LEFT_SPEED,
        SpinShooterConstants.SUBWOOFER_RIGHT_SPEED
      )
    }
    cmd.name = "shooting subwoofer"
    return cmd
  }

  fun shootAnywhere(): Command {
    val cmd = this.run {
      shootPiece(
        SpinShooterConstants.ANYWHERE_LEFT_SPEED,
        SpinShooterConstants.ANYWHERE_RIGHT_SPEED
      )
    }
    cmd.name = "shooting anywhere speed"
    return cmd
  }

  fun shootPass(): Command {
    val cmd = this.run {
      shootPiece(
        SpinShooterConstants.PASS_LEFT_SPEED,
        SpinShooterConstants.PASS_RIGHT_SPEED
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
          SpinShooterConstants.ANYWHERE_LEFT_SPEED,
          SpinShooterConstants.ANYWHERE_RIGHT_SPEED
        )

        val distance = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(FieldConstants.fieldLength - 14.144, 4.211))))
        } else {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(14.144, 4.211))))
        }

        val angle = Units.degreesToRadians(SpinShooterConstants.equation(distance) + 0.45)

        robot.pivot.moveToAngleSlow(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

        val fieldToRobot = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Translation2d(FieldConstants.fieldLength - 14.144, 4.211)
        } else {
          Translation2d(14.144, 4.211)
        }

        val robotToPoint = FieldConstants.SPEAKER_POSE - fieldToRobot

        robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        if (robot.shooter.atSetpoint() &&
          robot.driveCommand.atGoal &&
          robot.pivot.inTolerance() &&
          robot.mechController.hid.leftBumper
        ) {
          robot.feeder.intakeVoltage()
        }
      },
      { },
      { !robot.driveController.hid.yButton },
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
          SpinShooterConstants.ANYWHERE_LEFT_SPEED,
          SpinShooterConstants.ANYWHERE_RIGHT_SPEED
        )

        val distance = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(2.49, Units.inchesToMeters(218.42)))))
        } else {
          Units.metersToInches(abs(FieldConstants.SPEAKER_POSE.getDistance(Translation2d(FieldConstants.fieldLength - 2.49, Units.inchesToMeters(218.42)))))
        }

        val angle = Units.degreesToRadians(SpinShooterConstants.equation(distance))

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
        shootPiece(
          SpinShooterConstants.PASS_LEFT_SPEED,
          SpinShooterConstants.PASS_RIGHT_SPEED
        )

        robot.pivot.moveToAngleSlow(-0.75)

        val fieldToRobot = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
          Translation2d(FieldConstants.fieldLength - 7.5, 1.585)
        } else {
          Translation2d(7.5, 1.585)
        }

        val robotToPoint = FieldConstants.PASS_POSE - fieldToRobot

        robot.driveCommand.snapToAngle(robotToPoint.angle.radians + if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

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
      { !robot.driveController.hid.xButton },
      this,
      robot.pivot,
      robot.feeder,
      robot.undertaker
    )
    cmd.name = "pass shot"
    return cmd
  }

  fun autoAim(): Command {
    val cmd = FunctionalCommand(
      { },
      {
        shootPiece(
          SpinShooterConstants.ANYWHERE_LEFT_SPEED,
          SpinShooterConstants.ANYWHERE_RIGHT_SPEED
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

          val shotTime = SpinShooterConstants.TIME_MAP.get(distance)

          virtualTarget =
            Translation2d(
              FieldConstants.SPEAKER_POSE.x - shotTime * fieldRelSpeed.vxMetersPerSecond,
              FieldConstants.SPEAKER_POSE.y - shotTime * fieldRelSpeed.vyMetersPerSecond,
            )

          val newShotTime = SpinShooterConstants.TIME_MAP.get(distance)

          if (shotTime - newShotTime < 1e-3) {
            break
          }
        }

        val distance = abs(virtualTarget.getDistance(robot.drive.pose.translation))

        val angle = if (distance <= 1.30) 0.0 else SpinShooterConstants.SHOOTING_MAP.get(distance)

        robot.pivot.moveToAngleAuto(MathUtil.clamp(angle, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE))

        val robotToPoint = virtualTarget - robot.drive.pose.translation

        val desiredAngle = robotToPoint.angle + Rotation2d(if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) PI else 0.0)

        robot.driveCommand.snapToAngle(desiredAngle.radians)

        if (robot.shooter.atAimSetpoint() &&
          robot.driveCommand.atGoal &&
          robot.pivot.inShootAnywhereTolerance(angle) &&
          robot.mechController.hid.leftBumper
        ) {
          robot.feeder.intakeVoltage()
        }
      },
      { },
      {
        !robot.driveController.hid.bButton
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
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterConstants.IN_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterConstants.IN_TOLERANCE
  }

  fun atAimSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterConstants.AIM_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterConstants.AIM_TOLERANCE
  }

  fun atAutoSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterConstants.AUTO_SHOOT_TOL &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterConstants.AUTO_SHOOT_TOL &&
      desiredVels.first != 0.0 &&
      desiredVels.second != 0.0
  }

  fun atAmpSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < SpinShooterConstants.AMP_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < SpinShooterConstants.AMP_TOLERANCE
  }

  fun scoreAmp(): Command {
    val cmd = this.run {
      shootPiece(SpinShooterConstants.AMP_SPEED, SpinShooterConstants.AMP_SPEED)
    }
    cmd.name = "scoring amp"
    return cmd
  }

  fun duringIntake(): Command {
    val cmd = this.run {
      shootPiece(SpinShooterConstants.OUTTAKE_SPEED, SpinShooterConstants.OUTTAKE_SPEED)
    }
    cmd.name = "during intake"
    return cmd
  }

  fun shootPiece(leftSpeed: Double, rightSpeed: Double) {
    if (DriverStation.isDisabled()) {
      correct()
    } else {
      desiredVels = Pair(leftSpeed, rightSpeed)

      correct()
      predict()

      if (abs(rightVelocity.get() - rightSpeed) > SpinShooterConstants.START_INPT_ERR) {
        rightObserver.setXhat(1, 0.0)
      }

      if (abs(leftVelocity.get() - leftSpeed) > SpinShooterConstants.START_INPT_ERR) {
        leftObserver.setXhat(1, 0.0)
      }

      val voltages = getVoltages()

      rightMotor.setVoltage(voltages.second)
      leftMotor.setVoltage(voltages.first)
    }
  }

  fun coast(): Command {
    val cmd = this.runOnce {
      desiredVels = Pair(0.0, 0.0)

      correct()

      leftObserver.setXhat(1, 0.0)
      rightObserver.setXhat(1, 0.0)

      leftMotor.setVoltage(0.0)
      rightMotor.setVoltage(0.0)
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
        abs(leftVelocity.get() - desiredVels.first) < SpinShooterConstants.MIN_COAST_VEL &&
          abs(rightVelocity.get() - desiredVels.second) < SpinShooterConstants.MIN_COAST_VEL
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
        abs(leftVelocity.get()) < SpinShooterConstants.MIN_COAST_VEL &&
          abs(rightVelocity.get()) < SpinShooterConstants.MIN_COAST_VEL
      }.andThen(
        coast()
      )
    )
    cmd.name = "active stop"
    return cmd
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Right Voltage", { rightMotor.lastVoltage }, null)
    builder.addDoubleProperty("1.2 Last Left Voltage", { leftMotor.lastVoltage }, null)
    builder.publishConstString("2.0", "Current and Desired Velocities")
    builder.addDoubleProperty("2.1 Left Current Speed", { leftVelocity.get() }, null)
    builder.addDoubleProperty("2.2 Right Current Speed", { rightVelocity.get() }, null)
    builder.addDoubleProperty("2.3 Left Desired Speed", { desiredVels.first }, null)
    builder.addDoubleProperty("2.4 Right Desired Speed", { desiredVels.second }, null)
    builder.publishConstString("3.0", "Velocity Errors")
    builder.addDoubleProperty("3.1 Left Vel Error Pred", { leftObserver.getXhat(0) - desiredVels.first }, null)
    builder.addDoubleProperty("3.2 Right Vel Error Pred", { rightObserver.getXhat(0) - desiredVels.second }, null)
    builder.addDoubleProperty("3.3 Left Vel Error", { leftVelocity.get() - desiredVels.first }, null)
    builder.addDoubleProperty("3.4 Right Vel Error", { rightVelocity.get() - desiredVels.second }, null)
    builder.addBooleanProperty("3.5 In tolerance", ::atAimSetpoint, null)
    builder.publishConstString("4.0", "Encoder Positions")
    builder.addDoubleProperty("4.1 Left Enc Pos", { leftMotor.position }, null)
    builder.addDoubleProperty("4.2 Left Enc Pos", { rightMotor.position }, null)
    builder.publishConstString("5.0", "Input Err Estimation")
    builder.addDoubleProperty("5.1 Left Inpt Err Voltage", { -leftObserver.getXhat(1) }, null)
    builder.addDoubleProperty("5.2 Right Inpt Err Voltage", { -rightObserver.getXhat(1) }, null)
    builder.publishConstString("6.0", "Shoot from Anywhere")
    builder.addDoubleProperty("6.1 Speaker Distance (meters)", { abs(FieldConstants.SPEAKER_POSE.getDistance(robot.drive.pose.translation)) }, null)
  }

  companion object {
    fun createSpinShooter(robot: Robot): SpinShooter {
      val rightMotor = createSparkMax(
        "Shooter Right Motor",
        SpinShooterConstants.RIGHT_MOTOR_ID,
        encCreator = NEOEncoder.creator(
          SpinShooterConstants.UPR,
          SpinShooterConstants.GEARING,
          measurementPeriod = SpinShooterConstants.INTERNAL_MEASUREMENT_PD,
          depth = SpinShooterConstants.INTERNAL_ENC_DEPTH
        ),
        inverted = SpinShooterConstants.RIGHT_MOTOR_INVERTED,
        currentLimit = SpinShooterConstants.CURRENT_LIMIT,
        secondaryCurrentLimit = SpinShooterConstants.SECONDARY_CURRENT_LIMIT,
        enableBrakeMode = SpinShooterConstants.BRAKE_MODE
      )

      val leftMotor = createSparkMax(
        "Shooter Right Motor",
        SpinShooterConstants.LEFT_MOTOR_ID,
        encCreator = NEOEncoder.creator(
          SpinShooterConstants.UPR,
          SpinShooterConstants.GEARING,
          measurementPeriod = SpinShooterConstants.INTERNAL_MEASUREMENT_PD,
          depth = SpinShooterConstants.INTERNAL_ENC_DEPTH
        ),
        inverted = SpinShooterConstants.LEFT_MOTOR_INVERTED,
        currentLimit = SpinShooterConstants.CURRENT_LIMIT,
        secondaryCurrentLimit = SpinShooterConstants.SECONDARY_CURRENT_LIMIT,
        enableBrakeMode = SpinShooterConstants.BRAKE_MODE
      )

      val leftPlant = LinearSystemId.identifyVelocitySystem(
        SpinShooterConstants.LEFT_KV,
        SpinShooterConstants.LEFT_KA
      )

      val rightPlant = LinearSystemId.identifyVelocitySystem(
        SpinShooterConstants.RIGHT_KV,
        SpinShooterConstants.RIGHT_KA
      )

      val leftPlantSim = LinearSystemId.identifyVelocitySystem(
        SpinShooterConstants.LEFT_KV,
        SpinShooterConstants.LEFT_KA
      )

      val rightPlantSim = LinearSystemId.identifyVelocitySystem(
        SpinShooterConstants.RIGHT_KV,
        SpinShooterConstants.RIGHT_KA
      )

      val leftObserver = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        LinearSystem(
          MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            -SpinShooterConstants.LEFT_KV / SpinShooterConstants.LEFT_KA,
            SpinShooterConstants.LEFT_KA.pow(-1.0),
            0.0,
            0.0
          ),
          MatBuilder.fill(
            Nat.N2(),
            Nat.N1(),
            SpinShooterConstants.LEFT_KA.pow(-1.0),
            0.0
          ),
          MatBuilder.fill(
            Nat.N1(),
            Nat.N2(),
            1.0,
            0.0
          ),
          Matrix(Nat.N1(), Nat.N1())
        ),
        VecBuilder.fill(SpinShooterConstants.MODEL_VEL_STDDEV, SpinShooterConstants.INPT_ERR_STDDEV),
        VecBuilder.fill(SpinShooterConstants.ENCODER_VEL_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val rightObserver = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        LinearSystem(
          MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            -SpinShooterConstants.RIGHT_KV / SpinShooterConstants.RIGHT_KA,
            SpinShooterConstants.RIGHT_KA.pow(-1.0),
            0.0,
            0.0
          ),
          MatBuilder.fill(
            Nat.N2(),
            Nat.N1(),
            SpinShooterConstants.RIGHT_KA.pow(-1.0),
            0.0
          ),
          MatBuilder.fill(
            Nat.N1(),
            Nat.N2(),
            1.0,
            0.0
          ),
          Matrix(Nat.N1(), Nat.N1())
        ),
        VecBuilder.fill(SpinShooterConstants.MODEL_VEL_STDDEV, SpinShooterConstants.INPT_ERR_STDDEV),
        VecBuilder.fill(SpinShooterConstants.ENCODER_VEL_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val leftController = LinearQuadraticRegulator(
        leftPlant,
        VecBuilder.fill(SpinShooterConstants.LQR_VEL_TOL),
        VecBuilder.fill(SpinShooterConstants.LQR_MAX_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val rightController = LinearQuadraticRegulator(
        rightPlant,
        VecBuilder.fill(SpinShooterConstants.LQR_VEL_TOL),
        VecBuilder.fill(SpinShooterConstants.LQR_MAX_VOLTS),
        RobotConstants.LOOP_TIME
      )

      leftController.latencyCompensate(leftPlant, RobotConstants.LOOP_TIME, SpinShooterConstants.ENCODER_DELAY)
      rightController.latencyCompensate(rightPlant, RobotConstants.LOOP_TIME, SpinShooterConstants.ENCODER_DELAY)

      val leftFeedforward = LinearPlantInversionFeedforward(
        leftPlant,
        RobotConstants.LOOP_TIME
      )

      val rightFeedforward = LinearPlantInversionFeedforward(
        rightPlant,
        RobotConstants.LOOP_TIME
      )

      return if (RobotBase.isReal()) {
        SpinShooter(
          rightMotor,
          leftMotor,
          leftController,
          rightController,
          leftObserver,
          rightObserver,
          leftFeedforward,
          rightFeedforward,
          robot
        )
      } else {
        SpinShooterSim(
          rightMotor,
          leftMotor,
          leftController,
          rightController,
          leftObserver,
          rightObserver,
          leftFeedforward,
          rightFeedforward,
          leftPlantSim,
          rightPlantSim,
          robot
        )
      }
    }
  }
}
