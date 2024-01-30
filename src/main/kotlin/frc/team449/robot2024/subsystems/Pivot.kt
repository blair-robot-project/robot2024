package frc.team449.robot2024.subsystems

import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import java.util.function.Supplier

open class Pivot(
  private val motor: WrappedMotor,
  private val loop: LinearSystemLoop<N2, N1, N1>,
  private val profile: TrapezoidProfile,
  private val robot: Robot
) : SubsystemBase() {

  open val positionSupplier: Supplier<Double> =
    Supplier { motor.position }

  private var lastProfileReference = TrapezoidProfile.State(motor.position, motor.velocity)

  init {
    loop.reset(VecBuilder.fill(motor.position, motor.velocity))
    this.defaultCommand = hold()
  }

  fun hold(): Command {
    return this.run {
      loop.setNextR(lastProfileReference.position, lastProfileReference.velocity)
      loop.correct(VecBuilder.fill(positionSupplier.get()))
      loop.predict(RobotConstants.LOOP_TIME)

      motor.setVoltage(loop.getU(0))
    }
  }

  fun moveAmp(): Command {
    return this.run {
      moveToAngle(PivotConstants.AMP_ANGLE)
    }
  }

  fun moveSubwoofer(): Command {
    return this.run {
      moveToAngle(PivotConstants.SUBWOOFER_ANGLE)
    }
  }

  fun pivotShootAnywhere(): Command {
    return this.run {
      val distance = FieldConstants.SUBWOOFER_POSE.getDistance(robot.drive.pose.translation)
      val goal = ShooterConstants.SHOOTING_MAP.get(distance).get(2, 0)

      loop.setNextR(goal, 0.0)
      loop.correct(VecBuilder.fill(motor.position))
      loop.predict(RobotConstants.LOOP_TIME)

      motor.setVoltage(loop.getU(0))

			lastProfileReference = TrapezoidProfile.State(goal, 0.0)
    }
  }

  private fun moveToAngle(goal: Double) {
    lastProfileReference = profile.calculate(RobotConstants.LOOP_TIME, lastProfileReference, TrapezoidProfile.State(goal, 0.0))

    loop.setNextR(lastProfileReference.position, lastProfileReference.velocity)
    loop.correct(VecBuilder.fill(motor.position))
    loop.predict(RobotConstants.LOOP_TIME)

    motor.setVoltage(loop.getU(0))
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { motor.lastVoltage }, null)
  }

  companion object {
    fun createShooter(robot: Robot): Pivot {
      val motor = createSparkMax(
        "Shooter Right Motor",
        PivotConstants.MOTOR_ID,
        NEOEncoder.creator(
          PivotConstants.GEARING,
          PivotConstants.UPR
        ),
        inverted = PivotConstants.INVERTED,
        currentLimit = PivotConstants.CURRENT_LIM,
        slaveSparks = mapOf(Pair(PivotConstants.FOLLOWER_ID, PivotConstants.FOLLOWER_INVERTED))
      )

      val plant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor(
          MotorConstants.NOMINAL_VOLTAGE,
          MotorConstants.STALL_TORQUE,
          MotorConstants.STALL_CURRENT,
          MotorConstants.FREE_CURRENT,
          MotorConstants.FREE_SPEED,
          PivotConstants.NUM_MOTORS
        ),
        PivotConstants.MOMENT_OF_INERTIA,
        1 / PivotConstants.GEARING
      )

      val observer = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        plant,
        VecBuilder.fill(PivotConstants.MODEL_POS_DEVIATION, PivotConstants.MODEL_VEL_DEVIATION),
        VecBuilder.fill(PivotConstants.ENCODER_POS_DEVIATION),
        RobotConstants.LOOP_TIME
      )

      val controller = LinearQuadraticRegulator(
        plant,
        VecBuilder.fill(PivotConstants.POS_TOLERANCE, PivotConstants.VEL_TOLERANCE),
        VecBuilder.fill(PivotConstants.CONTROL_EFFORT_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val loop = LinearSystemLoop(
        plant,
        controller,
        observer,
        PivotConstants.MAX_VOLTAGE,
        RobotConstants.LOOP_TIME
      )

      val profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(
          PivotConstants.MAX_VELOCITY,
          PivotConstants.MAX_ACCEL
        )
      )

      return Pivot(motor, loop, profile, robot)
    }
  }
}
