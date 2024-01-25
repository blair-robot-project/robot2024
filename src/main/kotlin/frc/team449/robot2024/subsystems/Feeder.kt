package frc.team449.robot2024.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.subsystem.FeederConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Feeder(
  val motor: WrappedMotor
) : SubsystemBase() {

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(FeederConstants.INTAKE_VOLTAGE)
      motor.stopMotor();
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(FeederConstants.REVERSE_VOLTAGE)
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
    fun createProtoUndertaker(): Feeder {
      val motor = createSparkMax(
        "ProtoUndertaker Motor",
        FeederConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = FeederConstants.INVERTED,
        currentLimit = FeederConstants.CURRENT_LIM,
        slaveSparks = mapOf(Pair(FeederConstants.FOLLOLWER_ID, FeederConstants.FOLLOWER_INV))
      )

      return Feeder(motor)
    }
  }
}
