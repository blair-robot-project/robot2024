package frc.team449.robot2023.subsystems.elevator

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.ShooterConstants
import frc.team449.robot2024.subsystems.shooter.Shooter
import frc.team449.system.motor.WrappedMotor
import java.util.function.Supplier

class ElevatorSim(
  private val rightMotor: WrappedMotor,
  private val leftMotor: WrappedMotor,
  private val rightLoop: LinearSystemLoop<N1, N1, N1>,
  private val leftLoop: LinearSystemLoop<N1, N1, N1>,
  private val robot: Robot
) : Shooter(rightMotor, leftMotor, rightLoop, leftLoop, robot) {


  // State is (left velocity, right velocity)
  private var currentState = Pair(
    leftMotor.velocity * ShooterConstants.UPR * ShooterConstants.GEARING,
    rightMotor.velocity * ShooterConstants.UPR * ShooterConstants.GEARING
  )

  private val leftFlywheelSim = FlywheelSim(
    DCMotor(
      MotorConstants.NOMINAL_VOLTAGE,
      MotorConstants.STALL_TORQUE * ShooterConstants.EFFICIENCY,
      MotorConstants.STALL_CURRENT,
      MotorConstants.FREE_CURRENT,
      MotorConstants.FREE_SPEED,
      ShooterConstants.NUM_MOTORS
    ),
    ShooterConstants.GEARING,
    ShooterConstants.MOMENT_OF_INERTIA
  )

  private val rightFlywheelSim = FlywheelSim(
    DCMotor(
      MotorConstants.NOMINAL_VOLTAGE,
      MotorConstants.STALL_TORQUE * ShooterConstants.EFFICIENCY,
      MotorConstants.STALL_CURRENT,
      MotorConstants.FREE_CURRENT,
      MotorConstants.FREE_SPEED,
      ShooterConstants.NUM_MOTORS
    ),
    ShooterConstants.GEARING,
    ShooterConstants.MOMENT_OF_INERTIA
  )

  val leftVelocitySupplier =
    Supplier { leftFlywheelSim.angularVelocityRadPerSec }
  val rightVelocitySupplier =
    Supplier { rightFlywheelSim.angularVelocityRadPerSec }

  private var currentDraw = 0.0

  override fun periodic() {
    leftFlywheelSim.setInputVoltage(MathUtil.clamp(leftMotor.lastVoltage, -12.0, 12.0))
    rightFlywheelSim.setInputVoltage(MathUtil.clamp(rightMotor.lastVoltage, -12.0, 12.0))

    leftFlywheelSim.update(RobotConstants.LOOP_TIME)
    rightFlywheelSim.update(RobotConstants.LOOP_TIME)

    currentState = Pair(leftFlywheelSim.angularVelocityRPM, rightFlywheelSim.angularVelocityRPM)

    currentDraw = leftFlywheelSim.currentDrawAmps + rightFlywheelSim.currentDrawAmps
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)

    builder.publishConstString("4.0", "Current")
    builder.addDoubleProperty("4.1 Simulated current Draw", { currentDraw }, {})

    builder.publishConstString("5.0", "Velocity")
    builder.addDoubleProperty("5.1 Left Motor Vel", { currentState.first }, {})
    builder.addDoubleProperty("5.2 Right Motor Vel", { currentState.second }, {})
  }
}