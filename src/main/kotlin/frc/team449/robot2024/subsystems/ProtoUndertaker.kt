package frc.team449.robot2024.subsystems

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.subsystem.ProtoIntakeConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class ProtoUndertaker(
  private val motor: WrappedMotor
) : SubsystemBase() {

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(ProtoIntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(ProtoIntakeConstants.REVERSE_VOLTAGE)
    }
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { motor.lastVoltage }, null)
  }

  companion object {
    fun createProtoUndertaker(): ProtoUndertaker {
      val motor = createSparkMax(
        "ProtoUndertaker Motor",
        ProtoIntakeConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = ProtoIntakeConstants.INVERTED,
        currentLimit = ProtoIntakeConstants.CURRENT_LIM,
        slaveSparks = mapOf(Pair(ProtoIntakeConstants.FOLLOLWER_ID, ProtoIntakeConstants.FOLLOWER_INV))
      )

      return ProtoUndertaker(motor)
    }
  }
}
