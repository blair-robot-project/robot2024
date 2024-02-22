package frc.team449.robot2024.subsystems.pivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import frc.team449.system.motor.WrappedMotor
import java.util.function.Supplier

class PivotSim(
  private val motor: WrappedMotor,
  controller: LinearQuadraticRegulator<N2, N1, N1>,
  feedforward: LinearPlantInversionFeedforward<N2, N1, N1>,
  observer: KalmanFilter<N3, N1, N1>,
  profile: TrapezoidProfile,
  robot: Robot
) : Pivot(motor, controller, feedforward, observer, profile, robot) {

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
    true,
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

    builder.publishConstString("4.0", "Current")
    builder.addDoubleProperty("4.1 Simulated current Draw", { currentDraw }, {})

    builder.publishConstString("5.0", "Advantage Scope 3D Pos")
    builder.addDoubleArrayProperty(
      "5.1 3D Position",
      {
        val angle = Rotation3d(0.0, -currentState.first, 0.0)
        doubleArrayOf(
          -0.225,
          0.095,
          0.51,
          angle.quaternion.w,
          angle.quaternion.x,
          angle.quaternion.y,
          angle.quaternion.z,
        )
      },
      null
    )
  }
}
