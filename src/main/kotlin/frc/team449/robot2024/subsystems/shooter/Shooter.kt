package frc.team449.robot2024.subsystems.shooter

import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs

open class Shooter(
  private val rightMotor: WrappedMotor,
  private val leftMotor: WrappedMotor,
  private val rightLoop: LinearSystemLoop<N1, N1, N1>,
  private val leftLoop: LinearSystemLoop<N1, N1, N1>,
  private val robot: Robot
) : SubsystemBase() {

  /** TODO: Yo u needa make sure its only velocity control, no
   *    voltage control. */

  /** Left, Right Desired velocity */
  private var desiredVels = Pair(0.0, 0.0)

  open val rightVelocity: Supplier<Double> =
    Supplier { rightMotor.velocity }

  open val leftVelocity: Supplier<Double> =
    Supplier { leftMotor.velocity }

  private val leftRateLimiter = SlewRateLimiter(ShooterConstants.BRAKE_RATE_LIMIT)
  private val rightRateLimiter = SlewRateLimiter(ShooterConstants.BRAKE_RATE_LIMIT)

  init {
    // TODO: Uncomment this when finished with characterization
//    this.defaultCommand = updateOnly()
  }

  fun updateOnly(): Command {
    return this.run {
      leftLoop.correct(VecBuilder.fill(leftVelocity.get()))
      rightLoop.correct(VecBuilder.fill(rightVelocity.get()))
    }
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
    return abs(leftVelocity.get() - desiredVels.first) < ShooterConstants.LQR_VEL_TOL &&
      abs(rightVelocity.get() - desiredVels.second) < ShooterConstants.LQR_VEL_TOL &&
      desiredVels.first != 0.0 &&
      desiredVels.second != 0.0
  }

  fun scoreAmp(): Command {
    val cmd = this.runOnce {
      leftMotor.setVoltage(ShooterConstants.AMP_SCORE_VOLTAGE)
      rightMotor.setVoltage(ShooterConstants.AMP_SCORE_VOLTAGE)
    }
    cmd.name = "scoring amp"
    return cmd
  }

  fun duringIntake(): Command {
    val cmd = this.runOnce {
      rightMotor.setVoltage(ShooterConstants.DURING_INTAKE_VOLTAGE)
      leftMotor.setVoltage(ShooterConstants.DURING_INTAKE_VOLTAGE)
    }
    cmd.name = "during intake"
    return cmd
  }

  private fun shootPiece(rightSpeed: Double, leftSpeed: Double) {
    if (DriverStation.isDisabled()) {
      rightLoop.correct(VecBuilder.fill(rightVelocity.get()))
      leftLoop.correct(VecBuilder.fill(leftVelocity.get()))
    } else {
      rightLoop.nextR = VecBuilder.fill(rightSpeed)
      leftLoop.nextR = VecBuilder.fill(leftSpeed)

      desiredVels = Pair(leftSpeed, rightSpeed)

      rightLoop.correct(VecBuilder.fill(rightVelocity.get()))
      leftLoop.correct(VecBuilder.fill(leftVelocity.get()))

      rightLoop.predict(RobotConstants.LOOP_TIME)
      leftLoop.predict(RobotConstants.LOOP_TIME)

      rightMotor.setVoltage(rightLoop.getU(0) + ShooterConstants.RIGHT_KS)
      leftMotor.setVoltage(leftLoop.getU(0) + ShooterConstants.LEFT_KS)
    }
  }

  fun coast(): Command {
    val cmd = this.run {
      desiredVels = Pair(0.0, 0.0)
      leftMotor.setVoltage(0.0)
      rightMotor.setVoltage(0.0)
    }
    cmd.name = "coasting shooter"
    return cmd
  }

  fun forceStop(): Command {
    val cmd = this.run {
      desiredVels = Pair(0.0, 0.0)
      shootPiece(0.0, 0.0)
    }
    cmd.name = "force stop"
    return cmd
  }

  fun rampStop(): Command {
    val cmd = SequentialCommandGroup(
      this.runOnce {
        leftRateLimiter.reset(leftVelocity.get())
        rightRateLimiter.reset(rightVelocity.get())
      },
      this.run {
        desiredVels = Pair(leftRateLimiter.calculate(0.0), rightRateLimiter.calculate(0.0))
        shootPiece(desiredVels.first, desiredVels.second)
      }
    )
    cmd.name = "active stop"
    return cmd
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Right Voltage", { rightMotor.lastVoltage }, null)
    builder.addDoubleProperty("1.2 Last Left Voltage", { leftMotor.lastVoltage }, null)
    builder.publishConstString("2.0", "Current and Desired Velocities")
    builder.addDoubleProperty("2.1 Left Current Speed", { leftVelocity.get() * 60 / (2 * PI) }, null)
    builder.addDoubleProperty("2.2 Right Current Speed", { rightVelocity.get() * 60 / (2 * PI) }, null)
    builder.addDoubleProperty("2.3 Left Desired Speed", { desiredVels.first }, null)
    builder.addDoubleProperty("2.4 Right Desired Speed", { desiredVels.second }, null)
    builder.publishConstString("3.0", "Velocity Errors")
    builder.addDoubleProperty("3.1 Left Vel Error Pred", { leftLoop.error.get(0, 0) }, null)
    builder.addDoubleProperty("3.2 Right Vel Error Pred", { rightLoop.error.get(0, 0) }, null)
    builder.addDoubleProperty("3.3 Left Vel Error", { leftVelocity.get() - desiredVels.first }, null)
    builder.addDoubleProperty("3.4 Right Vel Error", { rightVelocity.get() - desiredVels.second }, null)
  }

  companion object {
    fun createShooter(robot: Robot): Shooter {
      val rightMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.RIGHT_MOTOR_ID,
        encCreator = NEOEncoder.creator(
          ShooterConstants.UPR,
          ShooterConstants.GEARING
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
          ShooterConstants.GEARING
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
        Nat.N1(),
        Nat.N1(),
        leftPlant,
        VecBuilder.fill(ShooterConstants.MODEL_VEL_STDDEV),
        VecBuilder.fill(ShooterConstants.ENCODER_VEL_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val rightObserver = KalmanFilter(
        Nat.N1(),
        Nat.N1(),
        rightPlant,
        VecBuilder.fill(ShooterConstants.MODEL_VEL_STDDEV),
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

      val rightLoop = LinearSystemLoop(
        rightPlant,
        rightController,
        rightObserver,
        ShooterConstants.MAX_VOLTAGE,
        RobotConstants.LOOP_TIME
      )

      val leftLoop = LinearSystemLoop(
        leftPlant,
        leftController,
        leftObserver,
        ShooterConstants.MAX_VOLTAGE,
        RobotConstants.LOOP_TIME
      )

      return if (RobotBase.isReal()) Shooter(rightMotor, leftMotor, rightLoop, leftLoop, robot) else ShooterSim(rightMotor, leftMotor, rightLoop, leftLoop, robot, rightPlant, leftPlant)
    }
  }
}
