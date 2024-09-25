package frc.team449.robot2024.subsystems.shooter

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.ChassisReference
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.SpinShooterKrakenConstants
import java.util.function.Supplier
import kotlin.math.PI

class SpinShooterKrakenSim(
  private val rightMotorThing: TalonFX,
  private val leftMotorThing: TalonFX,
  robot: Robot
) : SpinShooterKraken(rightMotorThing, leftMotorThing, robot) {

  private val leftFlywheelSim = FlywheelSim(
    LinearSystemId.identifyVelocitySystem(
      SpinShooterKrakenConstants.LEFT_KV / (2 * PI),
      SpinShooterKrakenConstants.LEFT_KA / (2 * PI)
    ),
    DCMotor.getKrakenX60(1),
    SpinShooterKrakenConstants.GEARING,
    VecBuilder.fill(2.15)
  )

  private val rightFlywheelSim = FlywheelSim(
    LinearSystemId.identifyVelocitySystem(
      SpinShooterKrakenConstants.RIGHT_KV / (2 * PI),
      SpinShooterKrakenConstants.RIGHT_KA / (2 * PI)
    ),
    DCMotor.getKrakenX60(1),
    SpinShooterKrakenConstants.GEARING,
    VecBuilder.fill(2.15)
  )

  override val leftVelocity: Supplier<Double> =
    Supplier { Units.radiansToRotations(leftFlywheelSim.angularVelocityRadPerSec) }

  override val rightVelocity: Supplier<Double> =
    Supplier { Units.radiansToRotations(rightFlywheelSim.angularVelocityRadPerSec) }

  private var currentDraw = 0.0

  private var rightMotorSim = rightMotorThing.simState
  private var leftMotorSim = leftMotorThing.simState

  override fun periodic() {
    rightMotorSim = rightMotorThing.simState
    leftMotorSim = leftMotorThing.simState

    rightMotorSim.Orientation = ChassisReference.CounterClockwise_Positive
    leftMotorSim.Orientation = ChassisReference.Clockwise_Positive

    leftMotorSim.setSupplyVoltage(12.0)
    rightMotorSim.setSupplyVoltage(12.0)

    currentDraw = rightMotorSim.supplyCurrent + leftMotorSim.supplyCurrent

    // Very simple current limiting
    if (currentDraw > 120) {
      leftFlywheelSim.setInputVoltage(MathUtil.clamp(leftMotorSim.motorVoltage, -12.0, 12.0) / (currentDraw / 90.0))
      rightFlywheelSim.setInputVoltage(MathUtil.clamp(rightMotorSim.motorVoltage, -12.0, 12.0) / (currentDraw / 90.0))
    } else {
      leftFlywheelSim.setInputVoltage(MathUtil.clamp(leftMotorSim.motorVoltage, -12.0, 12.0))
      rightFlywheelSim.setInputVoltage(MathUtil.clamp(rightMotorSim.motorVoltage, -12.0, 12.0))
    }

    leftFlywheelSim.update(RobotConstants.LOOP_TIME)
    rightFlywheelSim.update(RobotConstants.LOOP_TIME)

    leftMotorSim.setRotorVelocity(leftVelocity.get() * SpinShooterKrakenConstants.GEARING)

    rightMotorSim.setRotorVelocity(rightVelocity.get() * SpinShooterKrakenConstants.GEARING)
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)
    builder.publishConstString("6.0", "Current")
    builder.addDoubleProperty("6.1 Simulated current Draw", { currentDraw }, {})
  }
}
