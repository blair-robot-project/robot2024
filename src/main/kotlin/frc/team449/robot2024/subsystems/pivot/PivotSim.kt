package frc.team449.robot2024.subsystems.pivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedNEO
import java.util.function.Supplier

class PivotSim(
  simmedMotor: WrappedNEO,
  encoder: QuadEncoder,
  controller: LinearQuadraticRegulator<N2, N1, N1>,
  fastController: LinearQuadraticRegulator<N2, N1, N1>,
  autoController: LinearQuadraticRegulator<N2, N1, N1>,
  feedforward: LinearPlantInversionFeedforward<N2, N1, N1>,
  observer: KalmanFilter<N3, N1, N1>,
  profile: TrapezoidProfile
) : Pivot(simmedMotor, encoder, controller, fastController, autoController, feedforward, observer, profile) {

  private var currentState = Pair(0.0, 0.0)

  private val pivotSim = SingleJointedArmSim(
    DCMotor(
      MotorConstants.NOMINAL_VOLTAGE,
      MotorConstants.STALL_TORQUE * PivotConstants.EFFICIENCY,
      MotorConstants.STALL_CURRENT,
      MotorConstants.FREE_CURRENT,
      MotorConstants.FREE_SPEED,
      PivotConstants.NUM_MOTORS
    ),
    1 / PivotConstants.GEARING,
    PivotConstants.MOMENT_OF_INERTIA,
    PivotConstants.ARM_LENGTH,
    PivotConstants.MIN_ANGLE,
    PivotConstants.MAX_ANGLE,
    false,
    PivotConstants.MIN_ANGLE
  )

  override val positionSupplier =
    Supplier { pivotSim.angleRads }

  override val velocitySupplier =
    Supplier { pivotSim.velocityRadPerSec }

  private var currentDraw = 0.0

  override fun periodic() {
    pivotSim.setInputVoltage(MathUtil.clamp(motor.lastVoltage, -12.0, 12.0))

    pivotSim.update(RobotConstants.LOOP_TIME)

    currentState = Pair(pivotSim.angleRads, pivotSim.velocityRadPerSec)

    currentDraw = pivotSim.currentDrawAmps
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)

    builder.publishConstString("6.0", "Current")
    builder.addDoubleProperty("6.1 Simulated current Draw", { currentDraw }, {})
  }
}
