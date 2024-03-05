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
import frc.team449.robot2024.constants.RobotConstants
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
  val motor: WrappedMotor,
  private val controller: LinearQuadraticRegulator<N1, N1, N1>,
  private val observer: KalmanFilter<N2, N1, N1>,
  private val feedforward: LinearPlantInversionFeedforward<N1, N1, N1>
) : SubsystemBase() {

  /** Desired velocity */
  private var desiredVel = 0.0

  open val velocity: Supplier<Double> =
    Supplier { motor.velocity }

  private val rateLimiter = SlewRateLimiter(ShooterConstants.BRAKE_RATE_LIMIT)

  init {
    controller.reset()

    feedforward.reset(
      VecBuilder.fill(
        desiredVel
      )
    )

    this.defaultCommand = coast()
  }

  private fun correct() {
    val voltage = getVoltage()

    observer.correct(
      VecBuilder.fill(voltage),
      VecBuilder.fill(velocity.get())
    )
  }

  private fun predict() {
    val voltage = controller.calculate(
      VecBuilder.fill(
        observer.getXhat(0)
      ),
      VecBuilder.fill(
        desiredVel
      )
    ).plus(
      feedforward.calculate(
        VecBuilder.fill(
          desiredVel
        )
      )
    ).plus(
      -observer.getXhat(1)
    )

    observer.predict(voltage, RobotConstants.LOOP_TIME)
  }

  private fun getVoltage(): Double {
    val voltage = MathUtil.clamp(
      controller.getU(0) +
        feedforward.getUff(0) -
        observer.getXhat(1) +
        sign(desiredVel) * ShooterConstants.KS,
      -ShooterConstants.MAX_VOLTAGE,
      ShooterConstants.MAX_VOLTAGE
    )

    return voltage
  }

  fun hold(): Command {
    return this.run {
      shootPiece(
        desiredVel
      )
    }
  }

  fun setVoltage(volts: Double) {
    motor.setVoltage(volts)
  }

  fun shootSubwoofer(): Command {
    val cmd = this.run {
      shootPiece(
        ShooterConstants.SUBWOOFER_SPEED,
      )
    }
    cmd.name = "shooting subwoofer"
    return cmd
  }

  fun shootAuto(): Command {
    val cmd = this.run {
      shootPiece(
        ShooterConstants.AUTO_SPEED,
      )
    }
    cmd.name = "shooting subwoofer auto side"
    return cmd
  }

  fun atSetpoint(): Boolean {
    return abs(velocity.get() - desiredVel) < ShooterConstants.IN_TOLERANCE &&
      desiredVel != 0.0
  }

  fun atAutoSetpoint(): Boolean {
    return abs(velocity.get() - desiredVel) < ShooterConstants.AUTO_SHOOT_TOL &&
      desiredVel != 0.0
  }

  fun scoreAmp(): Command {
    val cmd = this.run {
      shootPiece(ShooterConstants.AMP_SPEED)
    }
    cmd.name = "scoring amp"
    return cmd
  }

  fun duringIntake(): Command {
    val cmd = this.run {
      shootPiece(ShooterConstants.OUTTAKE_SPEED)
    }
    cmd.name = "during intake"
    return cmd
  }

  private fun shootPiece(speed: Double) {
    if (DriverStation.isDisabled()) {
      correct()
    } else {
      desiredVel = speed

      correct()
      predict()

      motor.setVoltage(getVoltage())
    }
  }

  fun coast(): Command {
    val cmd = this.runOnce {
      desiredVel = 0.0

      correct()

      observer.setXhat(1, 0.0)

      motor.setVoltage(0.0)
    }
    cmd.name = "coasting shooter"
    return cmd
  }

  fun forceStop(): Command {
    val cmd = this.run {
      shootPiece(0.0)
    }
    cmd.name = "force stop"
    return ParallelDeadlineGroup(
      WaitUntilCommand {
        abs(velocity.get() - desiredVel) < ShooterConstants.MIN_COAST_VEL
      },
      cmd
    ).andThen(
      coast()
    )
  }

  fun rampStop(): Command {
    val cmd = SequentialCommandGroup(
      this.runOnce {
        rateLimiter.reset(velocity.get())
      },
      this.run {
        shootPiece(
          rateLimiter.calculate(0.0)
        )
      }.until {
        abs(velocity.get()) < ShooterConstants.MIN_COAST_VEL
      }.andThen(
        coast()
      )
    )
    cmd.name = "active stop"
    return cmd
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { motor.lastVoltage }, null)
    builder.publishConstString("2.0", "Current and Desired Velocities")
    builder.addDoubleProperty("2.1 Current Speed", { velocity.get() }, null)
    builder.addDoubleProperty("2.2 Desired Speed", { desiredVel }, null)
    builder.publishConstString("3.0", "Velocity Errors")
    builder.addDoubleProperty("3.1 Vel Error Pred", { observer.getXhat(0) - desiredVel }, null)
    builder.addDoubleProperty("3.2 Vel Error", { velocity.get() - desiredVel }, null)
    builder.publishConstString("4.0", "Encoder Positions")
    builder.addDoubleProperty("4.1 Enc Pos", { motor.position }, null)
    builder.publishConstString("5.0", "Input Err Estimation")
    builder.addDoubleProperty("5.1 Inpt Err Voltage", { -observer.getXhat(1) }, null)
  }

  companion object {
    fun createShooter(): Shooter {
      val motor = createSparkMax(
        "Shooter Motor",
        ShooterConstants.RIGHT_MOTOR_ID,
        encCreator = NEOEncoder.creator(
          ShooterConstants.UPR,
          ShooterConstants.GEARING,
          measurementPeriod = ShooterConstants.INTERNAL_MEASUREMENT_PD,
          depth = ShooterConstants.INTERNAL_ENC_DEPTH
        ),
        inverted = ShooterConstants.RIGHT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
        enableBrakeMode = ShooterConstants.BRAKE_MODE,
        slaveSparks = mapOf(Pair(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.LEFT_MOTOR_INVERTED_RELATIVE_TO_RIGHT))
      )

      val plant = LinearSystemId.identifyVelocitySystem(
        ShooterConstants.KV,
        ShooterConstants.KA
      )

      val plantSim = LinearSystemId.identifyVelocitySystem(
        ShooterConstants.KV + 0.0125,
        ShooterConstants.KA + 0.0065
      )

      val observer = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        LinearSystem(
          MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            -ShooterConstants.KV / ShooterConstants.KA,
            ShooterConstants.KA.pow(-1.0),
            0.0,
            0.0
          ),
          MatBuilder.fill(
            Nat.N2(),
            Nat.N1(),
            ShooterConstants.KA.pow(-1.0),
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

      val controller = LinearQuadraticRegulator(
        plant,
        VecBuilder.fill(ShooterConstants.LQR_VEL_TOL),
        VecBuilder.fill(ShooterConstants.LQR_MAX_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val feedforward = LinearPlantInversionFeedforward(
        plant,
        RobotConstants.LOOP_TIME
      )

      return if (RobotBase.isReal()) {
        Shooter(
          motor,
          controller,
          observer,
          feedforward
        )
      } else {
        ShooterSim(
          motor,
          controller,
          observer,
          feedforward,
          plantSim
        )
      }
    }
  }
}
