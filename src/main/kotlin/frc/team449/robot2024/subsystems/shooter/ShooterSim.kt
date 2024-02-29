package frc.team449.robot2024.subsystems.shooter

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.system.motor.WrappedMotor
import java.util.function.Supplier

class ShooterSim(
  motor: WrappedMotor,
  controller: LinearQuadraticRegulator<N1, N1, N1>,
  observer: KalmanFilter<N2, N1, N1>,
  feedforward: LinearPlantInversionFeedforward<N1, N1, N1>,
  plant: LinearSystem<N1, N1, N1>,
  robot: Robot
) : Shooter(motor, controller, observer, feedforward, robot) {

  private val flywheelSim = FlywheelSim(
    plant,
    DCMotor(
      MotorConstants.NOMINAL_VOLTAGE,
      MotorConstants.STALL_TORQUE * ShooterConstants.EFFICIENCY,
      MotorConstants.STALL_CURRENT,
      MotorConstants.FREE_CURRENT,
      MotorConstants.FREE_SPEED,
      ShooterConstants.NUM_MOTORS
    ),
    1 / ShooterConstants.GEARING,
  )

  override val velocity: Supplier<Double> =
    Supplier { flywheelSim.angularVelocityRadPerSec }

  private var currentDraw = 0.0

  override fun periodic() {
    flywheelSim.setInputVoltage(MathUtil.clamp(motor.lastVoltage, -12.0, 12.0))

    flywheelSim.update(RobotConstants.LOOP_TIME)

    currentDraw = ShooterConstants.NUM_MOTORS * flywheelSim.currentDrawAmps
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)
    builder.publishConstString("6.0", "Current")
    builder.addDoubleProperty("6.1 Simulated current Draw", { currentDraw }, {})
  }
}
