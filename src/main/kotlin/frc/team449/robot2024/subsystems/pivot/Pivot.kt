package frc.team449.robot2024.subsystems.pivot

import edu.wpi.first.math.MathUtil
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
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.sign

open class Pivot(
  private val motor: WrappedMotor,
  private val loop: LinearSystemLoop<N2, N1, N1>,
  private val profile: TrapezoidProfile,
  private val robot: Robot
) : SubsystemBase() {

  open val positionSupplier: Supplier<Double> =
    Supplier { motor.position }

  open val velocitySupplier: Supplier<Double> =
    Supplier { motor.velocity }

  private var lastProfileReference = TrapezoidProfile.State(0.0, 0.0)

  init {
    loop.reset(VecBuilder.fill(motor.position, motor.velocity))
    this.defaultCommand = hold()
  }

  fun hold(): Command {
    return this.run {
      lastProfileReference = TrapezoidProfile.State(lastProfileReference.position, 0.0)

      loop.setNextR(lastProfileReference.position, 0.0)
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

  fun autoAngle(): Command {
    return this.run {
      moveToAngle(PivotConstants.AUTO_ANGLE)
    }.until(::inTolerance)
  }

  fun autoStow(): Command {
    return this.run {
      moveToAngle(PivotConstants.STOW_ANGLE)
    }.until(::inTolerance)
  }

  fun inTolerance(): Boolean {
    return abs(positionSupplier.get() - lastProfileReference.position) < PivotConstants.POS_TOLERANCE &&
      abs(velocitySupplier.get()) < PivotConstants.MAX_VEL_TOL
  }

  fun manualMovement(axisSupplier: DoubleSupplier): Command {
    val cmd = this.run {
      moveToAngle(
        MathUtil.clamp(
          lastProfileReference.position + axisSupplier.asDouble * PivotConstants.MAX_VELOCITY * RobotConstants.LOOP_TIME / 4,
          PivotConstants.MIN_ANGLE,
          PivotConstants.MAX_ANGLE
        )
      )
    }
    cmd.name = "manual movement"
    return cmd
  }

  fun moveStow(): Command {
    return this.run {
      moveToAngle(PivotConstants.STOW_ANGLE)
    }
  }

  fun atSetpoint(): Boolean {
    return lastProfileReference.position - positionSupplier.get() < PivotConstants.POS_TOLERANCE
  }

  fun pivotShootAnywhere(): Command {
    return this.run {
      val distance = FieldConstants.SUBWOOFER_POSE.getDistance(robot.drive.pose.translation)
      val goal = ShooterConstants.SHOOTING_MAP.get(distance).get(2, 0)

      loop.setNextR(goal, 0.0)
      loop.correct(VecBuilder.fill(positionSupplier.get()))
      loop.predict(RobotConstants.LOOP_TIME)

      motor.setVoltage(loop.getU(0) + sign(lastProfileReference.velocity) * PivotConstants.KS)

      lastProfileReference = TrapezoidProfile.State(goal, 0.0)
    }
  }

  private fun moveToAngle(goal: Double) {
    lastProfileReference = profile.calculate(RobotConstants.LOOP_TIME, lastProfileReference, TrapezoidProfile.State(goal, 0.0))

    loop.setNextR(lastProfileReference.position, lastProfileReference.velocity)
    loop.correct(VecBuilder.fill(positionSupplier.get()))
    loop.predict(RobotConstants.LOOP_TIME)

    motor.setVoltage(loop.getU(0) + sign(lastProfileReference.velocity) * PivotConstants.KS)
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { motor.lastVoltage }, null)
    builder.publishConstString("2.0", "Position and Velocity")
    builder.addDoubleProperty("2.1 Current Position", { positionSupplier.get() }, null)
    builder.addDoubleProperty("2.2 Current Velocity", { velocitySupplier.get() }, null)
    builder.addDoubleProperty("2.3 Desired Position", { lastProfileReference.position }, null)
    builder.addDoubleProperty("2.4 Desired Velocity", { lastProfileReference.velocity }, null)
  }

  companion object {
    fun createPivot(robot: Robot): Pivot {
      val motor = createSparkMax(
        "Shooter Right Motor",
        PivotConstants.MOTOR_ID,
        encCreator = AbsoluteEncoder.creator(
          PivotConstants.ENC_CHANNEL,
          PivotConstants.OFFSET,
          PivotConstants.UPR,
          PivotConstants.ENC_INVERTED
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

      return if (RobotBase.isReal()) {
        Pivot(motor, loop, profile, robot)
      } else {
        PivotSim(motor, loop, plant, profile, robot)
      }
    }
  }
}
