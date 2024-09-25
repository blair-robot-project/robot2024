package frc.team449.robot2024.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.ClimberConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedNEO
import frc.team449.system.motor.createSparkMax
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.sign

class Climber(
  private val robot: Robot,
  private val rightMotor: WrappedNEO,
  private val leftMotor: WrappedNEO,
  private val controller: PIDController
) : SubsystemBase() {
  private var simCurrentPos = 0.0

  fun levelClimb(): Command {
    return PIDCommand(
      controller,
      { robot.drive.roll.degrees },
      { 0.0 },
      { value ->
        rightMotor.setVoltage(ClimberConstants.DEFAULT_PID_RETRACT + value)
        leftMotor.setVoltage(ClimberConstants.DEFAULT_PID_RETRACT - value)
      },
      this
    )
  }

  fun extend(): Command {
    return this.run {
      rightMotor.setVoltage(ClimberConstants.EXTEND_VOLTAGE)
      leftMotor.setVoltage(ClimberConstants.EXTEND_VOLTAGE)

//      if (rightMotor.position < ClimberConstants.MAX_ENC_POS) {
//        rightMotor.setVoltage(ClimberConstants.EXTEND_VOLTAGE)
//      } else {
//        rightMotor.setVoltage(0.0)
//      }
//
//      if (leftMotor.position < ClimberConstants.MAX_ENC_POS) {
//        leftMotor.setVoltage(ClimberConstants.EXTEND_VOLTAGE)
//      } else {
//        leftMotor.setVoltage(0.0)
//      }
    }
  }

  fun retract(): Command {
    return this.run {
      rightMotor.setVoltage(ClimberConstants.RETRACT_VOLTAGE)
      leftMotor.setVoltage(ClimberConstants.RETRACT_VOLTAGE)

//      if (rightMotor.position > ClimberConstants.MIN_ENC_POS) {
//        rightMotor.setVoltage(ClimberConstants.RETRACT_VOLTAGE)
//      } else {
//        rightMotor.setVoltage(0.0)
//      }
//
//      if (leftMotor.position > ClimberConstants.MIN_ENC_POS) {
//        leftMotor.setVoltage(ClimberConstants.RETRACT_VOLTAGE)
//      } else {
//        leftMotor.setVoltage(0.0)
//      }
    }
  }

  fun manual(axisLeft: DoubleSupplier, axisRight: DoubleSupplier): Command {
    return this.run {
      leftMotor.setVoltage(axisLeft.asDouble.pow(2) * sign(axisLeft.asDouble) * 12)
      rightMotor.setVoltage(axisRight.asDouble.pow(2) * sign(axisRight.asDouble) * 12)
    }
  }

  fun stop(): Command {
    return this.runOnce {
      rightMotor.stopMotor()
      leftMotor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Right Voltage", { rightMotor.lastVoltage }, null)
    builder.addDoubleProperty("1.2 Last Left Voltage", { leftMotor.lastVoltage }, null)
    builder.publishConstString("2.0", "Advantage Scope 3D Pos")
    builder.addDoubleArrayProperty(
      "2.1 3D Position",
      {
        doubleArrayOf(
          0.0,
          0.0,
          simCurrentPos,
          0.0,
          0.0,
          0.0,
          0.0
        )
      },
      null
    )
  }

  override fun simulationPeriodic() {
    super.simulationPeriodic()

    simCurrentPos += MathUtil.clamp(ClimberConstants.SIM_SPEED * RobotConstants.LOOP_TIME * sign(leftMotor.lastVoltage), 0.0, ClimberConstants.MAX_SIM_POS)
  }

  companion object {
    fun createClimber(robot: Robot): Climber {
      val rightMotor = createSparkMax(
        "ProtoUndertaker Motor",
        ClimberConstants.RIGHT_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = ClimberConstants.RIGHT_INVERTED,
        currentLimit = ClimberConstants.CURRENT_LIM,
      )

      val leftMotor = createSparkMax(
        "ProtoUndertaker Motor",
        ClimberConstants.LEFT_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = ClimberConstants.LEFT_INVERTED,
        currentLimit = ClimberConstants.CURRENT_LIM,
      )

      val controller = PIDController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD)

      return Climber(robot, rightMotor, leftMotor, controller)
    }
  }
}
