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
  rightMotor: WrappedMotor,
  leftMotor: WrappedMotor,
  leftController: LinearQuadraticRegulator<N1, N1, N1>,
  rightController: LinearQuadraticRegulator<N1, N1, N1>,
  leftObserver: KalmanFilter<N2, N1, N1>,
  rightObserver: KalmanFilter<N2, N1, N1>,
  leftFeedforward: LinearPlantInversionFeedforward<N1, N1, N1>,
  rightFeedforward: LinearPlantInversionFeedforward<N1, N1, N1>,
  leftPlant: LinearSystem<N1, N1, N1>,
  rightPlant: LinearSystem<N1, N1, N1>,
  robot: Robot
) : Shooter(rightMotor, leftMotor, leftController, rightController, leftObserver, rightObserver, leftFeedforward, rightFeedforward, robot) {

  private val leftFlywheelSim = FlywheelSim(
    leftPlant,
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

  private val rightFlywheelSim = FlywheelSim(
    rightPlant,
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

  override val leftVelocity: Supplier<Double> =
    Supplier { leftFlywheelSim.angularVelocityRadPerSec }

  override val rightVelocity: Supplier<Double> =
    Supplier { rightFlywheelSim.angularVelocityRadPerSec }

  private var currentDraw = 0.0

  override fun periodic() {
    leftFlywheelSim.setInputVoltage(MathUtil.clamp(leftMotor.lastVoltage, -12.0, 12.0))
    rightFlywheelSim.setInputVoltage(MathUtil.clamp(rightMotor.lastVoltage, -12.0, 12.0))

    leftFlywheelSim.update(RobotConstants.LOOP_TIME)
    rightFlywheelSim.update(RobotConstants.LOOP_TIME)

    currentDraw = leftFlywheelSim.currentDrawAmps + rightFlywheelSim.currentDrawAmps
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)
    builder.publishConstString("6.0", "Current")
    builder.addDoubleProperty("6.1 Simulated current Draw", { currentDraw }, {})
  }
}
