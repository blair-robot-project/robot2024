package frc.team449.robot2024.subsystems.shooter

import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import java.util.function.Supplier

open class Shooter(
  private val rightMotor: WrappedMotor,
  private val leftMotor: WrappedMotor,
  private val rightLoop: LinearSystemLoop<N1, N1, N1>,
  private val leftLoop: LinearSystemLoop<N1, N1, N1>,
  private val robot: Robot
) : SubsystemBase() {

  /** Left, Right Desired velocity */
  private var desiredVels = Pair(0.0, 0.0)

  open val rightVelocity: Supplier<Double> =
    Supplier { rightMotor.velocity }

  open val leftVelocity: Supplier<Double> =
    Supplier { leftMotor.velocity }

  init {
    this.defaultCommand = updateOnly()
  }

  fun updateOnly(): Command {
    return this.run {
      rightLoop.correct(VecBuilder.fill(rightVelocity.get()))
      leftLoop.correct(VecBuilder.fill(leftVelocity.get()))
    }
  }

  fun shootSubwoofer(): Command {
    return this.run {
      shootPiece(
        ShooterConstants.SUBWOOFER_RIGHT_SPEED,
        ShooterConstants.SUBWOOFER_LEFT_SPEED
      )
    }
  }

  fun shootAnywhere(): Command {
    return this.run {
      val distance = FieldConstants.SUBWOOFER_POSE.getDistance(robot.drive.pose.translation)

      val rightSpeed = ShooterConstants.SHOOTING_MAP.get(distance).get(0, 0)
      val leftSpeed = ShooterConstants.SHOOTING_MAP.get(distance).get(1, 0)

      desiredVels = Pair(leftSpeed, rightSpeed)

      shootPiece(rightSpeed, leftSpeed)
    }
  }

  fun atSetpoint(): Boolean {
    return leftVelocity.get() - desiredVels.first < ShooterConstants.LQR_VEL_TOL &&
      rightVelocity.get() - desiredVels.second < ShooterConstants.LQR_VEL_TOL
  }

  fun scoreAmp(): Command {
    return this.runOnce {
      leftMotor.setVoltage(ShooterConstants.AMP_SCORE_VOLTAGE)
      rightMotor.setVoltage(ShooterConstants.AMP_SCORE_VOLTAGE)
    }
  }

  fun duringIntake(): Command {
    return this.runOnce {
      rightMotor.setVoltage(ShooterConstants.DURING_INTAKE_VOLTAGE)
      leftMotor.setVoltage(ShooterConstants.DURING_INTAKE_VOLTAGE)
    }
  }

  private fun shootPiece(rightSpeed: Double, leftSpeed: Double) {
    if (DriverStation.isDisabled()) {
      rightLoop.correct(VecBuilder.fill(rightVelocity.get()))
      leftLoop.correct(VecBuilder.fill(leftVelocity.get()))
    } else {
      println(rightSpeed)
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

  fun stop(): Command {
    return this.runOnce {
      leftMotor.setVoltage(0.0)
      rightMotor.setVoltage(0.0)
    }
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
    builder.addDoubleProperty("3.1 Left Vel Error", { leftLoop.error.get(0, 0) }, null)
    builder.addDoubleProperty("3.2 Right Vel Error", { rightLoop.error.get(0, 0) }, null)
  }

  companion object {
    fun createShooter(robot: Robot): Shooter {
      val rightMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.RIGHT_MOTOR_ID,
        encCreator = QuadEncoder.creator(
          Encoder(ShooterConstants.RIGHT_CHANNEL_A, ShooterConstants.RIGHT_CHANNEL_B),
          ShooterConstants.CPR,
          ShooterConstants.UPR,
          ShooterConstants.GEARING,
          ShooterConstants.RIGHT_ENCODER_INVERTED
        ),
        inverted = ShooterConstants.RIGHT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
      )

      val leftMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.LEFT_MOTOR_ID,
        encCreator = QuadEncoder.creator(
          Encoder(ShooterConstants.LEFT_CHANNEL_A, ShooterConstants.LEFT_CHANNEL_B),
          ShooterConstants.CPR,
          ShooterConstants.UPR,
          ShooterConstants.GEARING,
          ShooterConstants.LEFT_ENCODER_INVERTED
        ),
        inverted = ShooterConstants.LEFT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
      )

      val leftPlant = LinearSystemId.createFlywheelSystem(
        DCMotor(
          MotorConstants.NOMINAL_VOLTAGE,
          MotorConstants.STALL_TORQUE,
          MotorConstants.STALL_CURRENT,
          MotorConstants.FREE_CURRENT,
          MotorConstants.FREE_SPEED,
          ShooterConstants.NUM_MOTORS
        ),
        ShooterConstants.MOMENT_OF_INERTIA,
        1 / ShooterConstants.GEARING
      )

      val rightPlant = LinearSystemId.createFlywheelSystem(
        DCMotor(
          MotorConstants.NOMINAL_VOLTAGE,
          MotorConstants.STALL_TORQUE,
          MotorConstants.STALL_CURRENT,
          MotorConstants.FREE_CURRENT,
          MotorConstants.FREE_SPEED,
          ShooterConstants.NUM_MOTORS
        ),
        ShooterConstants.MOMENT_OF_INERTIA,
        1 / ShooterConstants.GEARING
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
