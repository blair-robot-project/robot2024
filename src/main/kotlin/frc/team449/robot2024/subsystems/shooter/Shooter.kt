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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.encoder.NEOEncoder
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

  open val rightVelocity: Supplier<Double> =
    Supplier { rightMotor.velocity * ShooterConstants.UPR * ShooterConstants.GEARING }

  open val leftVelocity: Supplier<Double> =
    Supplier { leftMotor.velocity * ShooterConstants.UPR * ShooterConstants.GEARING }

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

      shootPiece(rightSpeed, leftSpeed)
    }
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
      rightLoop.setNextR(rightSpeed)
      leftLoop.setNextR(leftSpeed)

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
      leftMotor.stopMotor()
      rightMotor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Right Voltage", { rightMotor.lastVoltage }, null)
    builder.addDoubleProperty("1.2 Last Left Voltage", { leftMotor.lastVoltage }, null)
  }

  companion object {
    fun createShooter(robot: Robot): Shooter {
      val rightMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.RIGHT_MOTOR_ID,
        NEOEncoder.creator(
          ShooterConstants.GEARING,
          ShooterConstants.UPR
        ),
        inverted = ShooterConstants.RIGHT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
      )

      val leftMotor = createSparkMax(
        "Shooter Right Motor",
        ShooterConstants.LEFT_MOTOR_ID,
        NEOEncoder.creator(
          ShooterConstants.GEARING,
          ShooterConstants.UPR
        ),
        inverted = ShooterConstants.LEFT_MOTOR_INVERTED,
        currentLimit = ShooterConstants.CURRENT_LIMIT,
      )

      val plant = LinearSystemId.createFlywheelSystem(
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

      val observer = KalmanFilter(
        Nat.N1(),
        Nat.N1(),
        plant,
        VecBuilder.fill(ShooterConstants.MODEL_VEL_STDDEV),
        VecBuilder.fill(ShooterConstants.ENCODER_VEL_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val controller = LinearQuadraticRegulator(
        plant,
        VecBuilder.fill(ShooterConstants.LQR_VEL_TOL),
        VecBuilder.fill(ShooterConstants.LQR_MAX_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val rightLoop = LinearSystemLoop(
        plant,
        controller,
        observer,
        ShooterConstants.MAX_VOLTAGE,
        RobotConstants.LOOP_TIME
      )

      val leftLoop = LinearSystemLoop(
        plant,
        controller,
        observer,
        ShooterConstants.MAX_VOLTAGE,
        RobotConstants.LOOP_TIME
      )

      return Shooter(rightMotor, leftMotor, rightLoop, leftLoop, robot)
    }
  }
}
