package frc.team449.robot2024.subsystems.shooter

import edu.wpi.first.math.*
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import java.util.function.Supplier
import kotlin.Pair
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

open class Shooter(
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

  private val leftRateLimiter = SlewRateLimiter(ShooterConstants.BRAKE_RATE_LIMIT)
  private val rightRateLimiter = SlewRateLimiter(ShooterConstants.BRAKE_RATE_LIMIT)

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

    this.defaultCommand = updateOnly()
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
    ) // .plus(
//      -leftObserver.getXhat(1)
//    )

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
    ) // .plus(
//      -rightObserver.getXhat(1)
//    )

    rightObserver.predict(rightVoltage, RobotConstants.LOOP_TIME)
  }

  private fun getVoltages(): Pair<Double, Double> {
    val leftVoltage = MathUtil.clamp(
      leftController.getU(0) + leftFeedforward.getUff(0), // - leftObserver.getXhat(1),
      -ShooterConstants.MAX_VOLTAGE,
      ShooterConstants.MAX_VOLTAGE
    )

    val rightVoltage = MathUtil.clamp(
      rightController.getU(0) + rightFeedforward.getUff(0), // - rightObserver.getXhat(1),
      -ShooterConstants.MAX_VOLTAGE,
      ShooterConstants.MAX_VOLTAGE
    )

    return Pair(leftVoltage, rightVoltage)
  }

  fun updateOnly(): Command {
    return this.run {
      correct()
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
        ShooterConstants.SUBWOOFER_RIGHT_SPEED,
        ShooterConstants.SUBWOOFER_LEFT_SPEED
      )
    }
    cmd.name = "shooting subwoofer"
    return cmd
  }

  fun shootAuto(): Command {
    val cmd = this.run {
      shootPiece(
        ShooterConstants.AUTO_RIGHT_SPEED,
        ShooterConstants.AUTO_LEFT_SPEED
      )
    }
    cmd.name = "shooting subwoofer auto side"
    return cmd
  }

  fun shootAnywhere(): Command {
    val cmd = this.run {
      val distance = FieldConstants.SUBWOOFER_POSE.getDistance(robot.drive.pose.translation)

      val rightSpeed = ShooterConstants.SHOOTING_MAP.get(distance).get(0, 0)
      val leftSpeed = ShooterConstants.SHOOTING_MAP.get(distance).get(1, 0)

      desiredVels = Pair(leftSpeed, rightSpeed)

      shootPiece(rightSpeed, leftSpeed)
    }
    cmd.name = "shooting anywhere"
    return cmd
  }

  fun atSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < ShooterConstants.IN_TOLERANCE &&
      abs(rightVelocity.get() - desiredVels.second) < ShooterConstants.IN_TOLERANCE &&
      desiredVels.first != 0.0 &&
      desiredVels.second != 0.0
  }

  fun atAutoSetpoint(): Boolean {
    return abs(leftVelocity.get() - desiredVels.first) < ShooterConstants.AUTO_SHOOT_TOL &&
      abs(rightVelocity.get() - desiredVels.second) < ShooterConstants.AUTO_SHOOT_TOL &&
      desiredVels.first != 0.0 &&
      desiredVels.second != 0.0
  }

  fun scoreAmp(): Command {
    val cmd = this.run {
      shootPiece(ShooterConstants.AMP_SPEED, ShooterConstants.AMP_SPEED)
    }
    cmd.name = "scoring amp"
    return cmd
  }

  fun duringIntake(): Command {
    val cmd = this.run {
      shootPiece(ShooterConstants.OUTTAKE_SPEED, ShooterConstants.OUTTAKE_SPEED)
    }
    cmd.name = "during intake"
    return cmd
  }

  private fun shootPiece(rightSpeed: Double, leftSpeed: Double) {
    if (DriverStation.isDisabled()) {
      correct()
    } else {
      desiredVels = Pair(leftSpeed, rightSpeed)

      correct()
      predict()

      val voltages = getVoltages()

      rightMotor.setVoltage(voltages.second + sign(rightSpeed) * ShooterConstants.RIGHT_KS)
      leftMotor.setVoltage(voltages.first + sign(leftSpeed) * ShooterConstants.LEFT_KS)
    }
  }

  fun coast(): Command {
    val cmd = this.runOnce {
      desiredVels = Pair(0.0, 0.0)

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
        abs(leftVelocity.get() - desiredVels.first) < ShooterConstants.IN_TOLERANCE &&
          abs(rightVelocity.get() - desiredVels.second) < ShooterConstants.IN_TOLERANCE
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
          rightRateLimiter.calculate(0.0),
          leftRateLimiter.calculate(0.0)
        )
      }.until {
        abs(leftVelocity.get()) < ShooterConstants.MIN_RAMP_VEL &&
          abs(rightVelocity.get()) < ShooterConstants.MIN_RAMP_VEL
      }.andThen(
        forceStop()
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
    builder.publishConstString("4.0", "Encoder Positions")
    builder.addDoubleProperty("4.1 Left Enc Pos", { leftMotor.position }, null)
    builder.addDoubleProperty("4.2 Left Enc Pos", { rightMotor.position }, null)
    builder.publishConstString("5.0", "Input Err Estimation")
    builder.addDoubleProperty("5.1 Left Inpt Err Voltage", { -leftObserver.getXhat(1) }, null)
    builder.addDoubleProperty("5.2 Right Inpt Err Voltage", { -rightObserver.getXhat(1) }, null)
  }

  companion object {
    fun createShooter(robot: Robot): Shooter {
      val rightMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.RIGHT_MOTOR_ID,
        encCreator = NEOEncoder.creator(
          ShooterConstants.UPR,
          ShooterConstants.GEARING,
          measurementPeriod = ShooterConstants.INTERNAL_MEASUREMENT_PD,
          depth = ShooterConstants.INTERNAL_ENC_DEPTH
        ),
        inverted = ShooterConstants.RIGHT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
        enableBrakeMode = ShooterConstants.BRAKE_MODE
      )

      val leftMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.LEFT_MOTOR_ID,
        encCreator = NEOEncoder.creator(
          ShooterConstants.UPR,
          ShooterConstants.GEARING,
          measurementPeriod = ShooterConstants.INTERNAL_MEASUREMENT_PD,
          depth = ShooterConstants.INTERNAL_ENC_DEPTH
        ),
        inverted = ShooterConstants.LEFT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
        enableBrakeMode = ShooterConstants.BRAKE_MODE
      )

      val leftPlant = LinearSystemId.identifyVelocitySystem(
        ShooterConstants.LEFT_KV,
        ShooterConstants.LEFT_KA
      )

      val rightPlant = LinearSystemId.identifyVelocitySystem(
        ShooterConstants.RIGHT_KV,
        ShooterConstants.RIGHT_KA
      )

      val leftObserver = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        LinearSystem(
          MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            -ShooterConstants.LEFT_KV / ShooterConstants.LEFT_KA,
            ShooterConstants.LEFT_KA.pow(-1.0),
            0.0,
            0.0
          ),
          MatBuilder.fill(
            Nat.N2(),
            Nat.N1(),
            ShooterConstants.LEFT_KA.pow(-1.0),
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
        VecBuilder.fill(ShooterConstants.MODEL_VEL_STDDEV, ShooterConstants.INPT_ERR_STDDEV),
        VecBuilder.fill(ShooterConstants.ENCODER_VEL_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val rightObserver = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        LinearSystem(
          MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            -ShooterConstants.RIGHT_KV / ShooterConstants.RIGHT_KA,
            ShooterConstants.RIGHT_KA.pow(-1.0),
            0.0,
            0.0
          ),
          MatBuilder.fill(
            Nat.N2(),
            Nat.N1(),
            ShooterConstants.RIGHT_KA.pow(-1.0),
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
        VecBuilder.fill(ShooterConstants.MODEL_VEL_STDDEV, ShooterConstants.INPT_ERR_STDDEV),
        VecBuilder.fill(ShooterConstants.ENCODER_VEL_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val leftController = LinearQuadraticRegulator(
        leftPlant,
        VecBuilder.fill(ShooterConstants.LQR_VEL_TOL),
        VecBuilder.fill(ShooterConstants.LQR_MAX_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val rightController = LinearQuadraticRegulator(
        rightPlant,
        VecBuilder.fill(ShooterConstants.LQR_VEL_TOL),
        VecBuilder.fill(ShooterConstants.LQR_MAX_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val leftFeedforward = LinearPlantInversionFeedforward(
        leftPlant,
        RobotConstants.LOOP_TIME
      )

      val rightFeedforward = LinearPlantInversionFeedforward(
        rightPlant,
        RobotConstants.LOOP_TIME
      )

      return if (RobotBase.isReal()) {
        Shooter(
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
        ShooterSim(
          rightMotor,
          leftMotor,
          leftController,
          rightController,
          leftObserver,
          rightObserver,
          leftFeedforward,
          rightFeedforward,
          leftPlant,
          rightPlant,
          robot
        )
      }
    }
  }
}
