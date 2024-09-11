package frc.team449.robot2024.subsystems.pivot

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.*
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.system.encoder.QuadEncoder
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.pow

open class Pivot(
  val motor: CANSparkMax,
  val encoder: QuadEncoder,
  private val controller: LinearQuadraticRegulator<N2, N1, N1>,
  private val fastController: LinearQuadraticRegulator<N2, N1, N1>,
  private val autoController: LinearQuadraticRegulator<N2, N1, N1>,
  private val feedforward: LinearPlantInversionFeedforward<N2, N1, N1>,
  private val observer: KalmanFilter<N3, N1, N1>,
  private val profile: TrapezoidProfile
) : SubsystemBase() {

  private val slowProfile = TrapezoidProfile(
    TrapezoidProfile.Constraints(
      PivotConstants.MAX_VELOCITY,
      PivotConstants.SLOW_ACCEL
    )
  )

  private val autoProfile = TrapezoidProfile(
    TrapezoidProfile.Constraints(
      PivotConstants.MAX_VELOCITY,
      PivotConstants.AUTO_ACCEL
    )
  )

  open val positionSupplier: Supplier<Double> =
    Supplier { encoder.position }


  open val velocitySupplier: Supplier<Double> =
    Supplier { encoder.velocity }

  var lastProfileReference = TrapezoidProfile.State(
    PivotConstants.STOW_ANGLE,
    0.0
  )

  init {
    controller.reset()
    fastController.reset()
    autoController.reset()

    feedforward.reset(
      VecBuilder.fill(
        lastProfileReference.position,
        lastProfileReference.velocity
      )
    )

    observer.xhat = VecBuilder.fill(
      lastProfileReference.position,
      lastProfileReference.velocity,
      0.0
    )

    motor.inverted = PivotConstants.INVERTED
    motor.setSmartCurrentLimit(PivotConstants.CURRENT_LIM)
    val followerSpark = CANSparkMax(PivotConstants.FOLLOWER_ID, CANSparkLowLevel.MotorType.kBrushless)
    followerSpark.restoreFactoryDefaults()
    followerSpark.follow(motor, PivotConstants.FOLLOWER_INVERTED)
    followerSpark.idleMode = motor.idleMode
    followerSpark.setSmartCurrentLimit(PivotConstants.CURRENT_LIM)
    followerSpark.burnFlash()

    this.defaultCommand = hold()
  }

  /** Kalman Filter correct step */
  private fun correctSlow() {
    observer.correct(
      VecBuilder.fill(getSlowVoltage()),
      VecBuilder.fill(positionSupplier.get())
    )
  }

  private fun correctAuto() {
    observer.correct(
      VecBuilder.fill(getAutoVoltage()),
      VecBuilder.fill(positionSupplier.get())
    )
  }

  private fun correctFast() {
    observer.correct(
      VecBuilder.fill(getFastVoltage()),
      VecBuilder.fill(positionSupplier.get())
    )
  }

  /** Give the LQR the next goal and do the KF predict step*/
  private fun slowPredict() {
    val voltage = controller.calculate(
      VecBuilder.fill(
        observer.getXhat(0),
        observer.getXhat(1)
      ),
      VecBuilder.fill(
        lastProfileReference.position,
        lastProfileReference.velocity
      )
    ).plus(
      feedforward.calculate(
        VecBuilder.fill(
          lastProfileReference.position,
          lastProfileReference.velocity
        )
      )
    ).plus(
      -observer.getXhat(2)
    ).plus(
      PivotConstants.SIMPLE_FF.calculate(lastProfileReference.position + PivotConstants.KG_OFFSET, lastProfileReference.velocity)
    )

    observer.predict(voltage, RobotConstants.LOOP_TIME)
  }

  private fun fastPredict() {
    val voltage = fastController.calculate(
      VecBuilder.fill(
        observer.getXhat(0),
        observer.getXhat(1)
      ),
      VecBuilder.fill(
        lastProfileReference.position,
        lastProfileReference.velocity
      )
    ).plus(
      feedforward.calculate(
        VecBuilder.fill(
          lastProfileReference.position,
          lastProfileReference.velocity
        )
      )
    ).plus(
      -observer.getXhat(2)
    ).plus(
      PivotConstants.SIMPLE_FF.calculate(lastProfileReference.position + PivotConstants.KG_OFFSET, lastProfileReference.velocity)
    )

    observer.predict(voltage, RobotConstants.LOOP_TIME)
  }

  private fun autoPredict() {
    val voltage = autoController.calculate(
      VecBuilder.fill(
        observer.getXhat(0),
        observer.getXhat(1)
      ),
      VecBuilder.fill(
        lastProfileReference.position,
        lastProfileReference.velocity
      )
    ).plus(
      feedforward.calculate(
        VecBuilder.fill(
          lastProfileReference.position,
          lastProfileReference.velocity
        )
      )
    ).plus(
      -observer.getXhat(2)
    ).plus(
      PivotConstants.SIMPLE_FF.calculate(lastProfileReference.position + PivotConstants.KG_OFFSET, lastProfileReference.velocity)
    )

    observer.predict(voltage, RobotConstants.LOOP_TIME)
  }

  private fun getSlowVoltage(): Double {
    return MathUtil.clamp(
      controller.getU(0) +
        feedforward.getUff(0) -
        observer.getXhat(2) +
        PivotConstants.SIMPLE_FF.calculate(
          lastProfileReference.position + PivotConstants.KG_OFFSET,
          lastProfileReference.velocity
        ),
      -PivotConstants.MAX_VOLTAGE,
      PivotConstants.MAX_VOLTAGE
    )
  }

  private fun getFastVoltage(): Double {
    return MathUtil.clamp(
      fastController.getU(0) +
        feedforward.getUff(0) -
        observer.getXhat(2) +
        PivotConstants.SIMPLE_FF.calculate(
          lastProfileReference.position + PivotConstants.KG_OFFSET,
          lastProfileReference.velocity
        ),
      -PivotConstants.MAX_VOLTAGE,
      PivotConstants.MAX_VOLTAGE
    )
  }

  private fun getAutoVoltage(): Double {
    return MathUtil.clamp(
      autoController.getU(0) +
        feedforward.getUff(0) -
        observer.getXhat(2) +
        PivotConstants.SIMPLE_FF.calculate(
          lastProfileReference.position + PivotConstants.KG_OFFSET,
          lastProfileReference.velocity
        ),
      -PivotConstants.MAX_VOLTAGE,
      PivotConstants.MAX_VOLTAGE
    )
  }

  fun hold(): Command {
    return this.run {
      moveToAngleSlow(lastProfileReference.position)
      observer.setXhat(2, 0.0)
    }
  }

  fun moveAmp(): Command {
    return this.run {
      moveToAngleFast(PivotConstants.AMP_ANGLE)
    }
  }

  fun moveClimb(): Command {
    return this.run {
      moveToAngleSlow(PivotConstants.CLIMB_ANGLE)
    }
  }

  fun movePass(): Command {
    return this.run {
      moveToAngleSlow(PivotConstants.PASS_ANGLE)
    }
  }

  fun moveAngleCmd(angle: Double): Command {
    return this.run {
      moveToAngleSlow(angle)
    }
  }

  fun moveAngleCmdAuto(angle: Double): Command {
    return runOnce {
      lastProfileReference = TrapezoidProfile.State(
        MathUtil.clamp(lastProfileReference.position, PivotConstants.MIN_ANGLE, PivotConstants.MAX_ANGLE),
        lastProfileReference.velocity
      )
    }.andThen(
      run {
        moveToAngleAuto(angle)
      }
    )
  }

  fun inTolerance(): Boolean {
    return abs(positionSupplier.get() - lastProfileReference.position) < PivotConstants.MAX_POS_ERROR &&
      abs(velocitySupplier.get()) < PivotConstants.MAX_VEL_ERROR
  }

  fun inTolerance(pos: Double): Boolean {
    return abs(positionSupplier.get() - pos) < PivotConstants.MAX_POS_ERROR &&
      abs(velocitySupplier.get()) < PivotConstants.MAX_VEL_ERROR
  }

  fun inAutoTolerance(pos: Double): Boolean {
    return abs(positionSupplier.get() - pos) < PivotConstants.AUTO_MAX_POS_ERROR &&
      abs(velocitySupplier.get()) < PivotConstants.MAX_VEL_ERROR
  }

  fun inShootAnywhereTolerance(pos: Double): Boolean {
    return abs(positionSupplier.get() - pos) < PivotConstants.SHOOT_ANYWHERE_POS_TOLERANCE &&
      abs(velocitySupplier.get()) < PivotConstants.MAX_VEL_ERROR
  }

  fun inAmpTolerance(): Boolean {
    return abs(positionSupplier.get() - PivotConstants.AMP_ANGLE) < PivotConstants.AMP_TOL &&
      abs(velocitySupplier.get()) < PivotConstants.AMP_VEL_TOL
  }

  fun manualUp(): Command {
    val cmd = this.run {
      moveToAngleSlow(
        MathUtil.clamp(
          lastProfileReference.position + PivotConstants.MAX_VELOCITY * RobotConstants.LOOP_TIME / 3,
          PivotConstants.MIN_ANGLE,
          PivotConstants.MAX_ANGLE
        )
      )
    }
    cmd.name = "manual movement up"
    return cmd
  }

  fun manualDown(): Command {
    val cmd = this.run {
      moveToAngleSlow(
        MathUtil.clamp(
          lastProfileReference.position - PivotConstants.MAX_VELOCITY * RobotConstants.LOOP_TIME / 3,
          PivotConstants.MIN_ANGLE,
          PivotConstants.MAX_ANGLE
        )
      )
    }
    cmd.name = "manual movement down"
    return cmd
  }

  fun moveStow(): Command {
    return this.run {
      moveToAngleSlow(PivotConstants.STOW_ANGLE)
      observer.setXhat(2, 0.0)
    }
  }

  fun moveToAngleSlow(goal: Double) {
    lastProfileReference = slowProfile.calculate(RobotConstants.LOOP_TIME, lastProfileReference, TrapezoidProfile.State(goal, 0.0))

    correctSlow()
    slowPredict()

    motor.setVoltage(getSlowVoltage())

    if (abs(lastProfileReference.position - goal) > PivotConstants.START_INPT_ERR) {
      observer.setXhat(2, 0.0)
    }
  }

  fun moveToAngleAuto(goal: Double) {
    lastProfileReference = autoProfile.calculate(RobotConstants.LOOP_TIME, lastProfileReference, TrapezoidProfile.State(goal, 0.0))

    correctAuto()
    autoPredict()

    motor.setVoltage(getAutoVoltage())

    observer.setXhat(2, 0.0)
  }

  fun setVoltage(voltage: Double) {
    motor.setVoltage(voltage)
  }

  fun moveToAngleFast(goal: Double) {
    lastProfileReference = profile.calculate(RobotConstants.LOOP_TIME, lastProfileReference, TrapezoidProfile.State(goal, 0.0))

    correctFast()
    fastPredict()

    motor.setVoltage(getFastVoltage())

    if (abs(lastProfileReference.position - goal) > PivotConstants.START_INPT_ERR) {
      observer.setXhat(2, 0.0)
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { motor.get() }, null)
    builder.publishConstString("2.0", "Position and Velocity")
    builder.addDoubleProperty("2.1 Current Position", { positionSupplier.get() }, null)
    builder.addDoubleProperty("2.2 Current Velocity", { velocitySupplier.get() }, null)
    builder.addDoubleProperty("2.3 Desired Position", { lastProfileReference.position }, null)
    builder.addDoubleProperty("2.4 Desired Velocity", { lastProfileReference.velocity }, null)
    builder.addDoubleProperty("2.5 Error", { lastProfileReference.position - positionSupplier.get() }, null)
    builder.addDoubleProperty("2.6 Absolute Position", { motor.encoder.position }, null)
    builder.addDoubleProperty("2.7 Absolute Velocity", { motor.encoder.velocity }, null)
    builder.addBooleanProperty("2.8 In tolerance", ::inTolerance, null)
    builder.publishConstString("3.0", "State Space Stuff")
    builder.addDoubleProperty("3.1 Predicted Position", { observer.getXhat(0) }, null)
    builder.addDoubleProperty("3.2 Predicted Velocity", { observer.getXhat(1) }, null)
    builder.addDoubleProperty("3.3 Predicted Input Error", { -observer.getXhat(2) }, null)
    builder.addDoubleProperty("3.4 Predicted State Error", { lastProfileReference.position - observer.getXhat(0) }, null)
  }

  companion object {
    fun createPivot(): Pivot {
      val motor = CANSparkMax(
        PivotConstants.MOTOR_ID,
        CANSparkLowLevel.MotorType.kBrushless
      )

      val encoder = QuadEncoder(
        name = "Pivot Quad Encoder",
        encoder = PivotConstants.QUAD_ENCODER,
        encoderCPR = PivotConstants.CPR,
        unitPerRotation = PivotConstants.UPR,
        gearing = 1.0,
        samplesToAverage = PivotConstants.SAMPLES_TO_AVERAGE
      )

      val motorModel = DCMotor(
        MotorConstants.NOMINAL_VOLTAGE,
        MotorConstants.STALL_TORQUE,
        MotorConstants.STALL_CURRENT,
        MotorConstants.FREE_CURRENT,
        MotorConstants.FREE_SPEED,
        PivotConstants.NUM_MOTORS
      )

      val linearSystemIDPlant = LinearSystemId.createSingleJointedArmSystem(
        motorModel,
        PivotConstants.MOMENT_OF_INERTIA,
        1 / PivotConstants.GEARING
      )

      val A = MatBuilder.fill(
        Nat.N3(),
        Nat.N3(),
        0.0,
        1.0,
        0.0,
        0.0,
        -PivotConstants.GEARING.pow(-2.0) * motorModel.KtNMPerAmp /
          (motorModel.KvRadPerSecPerVolt * motorModel.rOhms * PivotConstants.MOMENT_OF_INERTIA),
        PivotConstants.GEARING.pow(-1.0) * motorModel.KtNMPerAmp /
          (motorModel.rOhms * PivotConstants.MOMENT_OF_INERTIA),
        0.0,
        0.0,
        0.0
      )

      val B = MatBuilder.fill(
        Nat.N3(),
        Nat.N1(),
        0.0,
        PivotConstants.GEARING.pow(-1.0) * motorModel.KtNMPerAmp / (motorModel.rOhms * PivotConstants.MOMENT_OF_INERTIA),
        0.0
      )

      val C = MatBuilder.fill(
        Nat.N1(),
        Nat.N3(),
        1.0,
        0.0,
        0.0
      )

      val D = Matrix(Nat.N1(), Nat.N1())

      val plant = LinearSystem(
        A,
        B,
        C,
        D
      )

      val observer = KalmanFilter(
        Nat.N3(),
        Nat.N1(),
        plant,
        VecBuilder.fill(
          PivotConstants.MODEL_POS_DEVIATION,
          PivotConstants.MODEL_VEL_DEVIATION,
          PivotConstants.MODEL_INPUT_ERROR_DEVIATION
        ),
        VecBuilder.fill(PivotConstants.ENCODER_POS_DEVIATION),
        RobotConstants.LOOP_TIME
      )

      val controller = LinearQuadraticRegulator(
        linearSystemIDPlant,
        VecBuilder.fill(PivotConstants.POS_TOLERANCE, PivotConstants.VEL_TOLERANCE),
        VecBuilder.fill(PivotConstants.CONTROL_EFFORT_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val fastController = LinearQuadraticRegulator(
        linearSystemIDPlant,
        VecBuilder.fill(PivotConstants.FAST_POS_TOLERANCE, PivotConstants.FAST_VEL_TOLERANCE),
        VecBuilder.fill(PivotConstants.CONTROL_EFFORT_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val autoController = LinearQuadraticRegulator(
        linearSystemIDPlant,
        VecBuilder.fill(PivotConstants.AUTO_POS_TOLERANCE, PivotConstants.AUTO_VEL_TOLERANCE),
        VecBuilder.fill(PivotConstants.CONTROL_EFFORT_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val feedforward = LinearPlantInversionFeedforward(
        linearSystemIDPlant,
        RobotConstants.LOOP_TIME
      )

      val profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(
          PivotConstants.MAX_VELOCITY,
          PivotConstants.MAX_ACCEL
        )
      )

      return if (RobotBase.isReal()) {
        Pivot(motor, encoder, controller, fastController, autoController, feedforward, observer, profile)
      } else {
        PivotSim(
          motor,
          encoder,
          controller,
          fastController,
          autoController,
          feedforward,
          observer,
          profile
        )
      }
    }
  }
}
