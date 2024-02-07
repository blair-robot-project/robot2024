package frc.team449.robot2024.subsystems.shooter

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
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
  private val rightMotor: WrappedMotor,
  private val leftMotor: WrappedMotor,
  rightLoop: LinearSystemLoop<N1, N1, N1>,
  leftLoop: LinearSystemLoop<N1, N1, N1>,
  robot: Robot,
  rightPlant: LinearSystem<N1, N1, N1>,
  leftPlant: LinearSystem<N1, N1, N1>
) : Shooter(rightMotor, leftMotor, rightLoop, leftLoop, robot) {

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
    ShooterConstants.GEARING,
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
    builder.publishConstString("4.0", "Current")
    builder.addDoubleProperty("4.1 Simulated current Draw", { currentDraw }, {})
  }
}
