package frc.team449.robot2024.subsystems

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.subsystem.UndertakerConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Undertaker(
  private val motor: WrappedMotor
) : SubsystemBase() {

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(UndertakerConstants.INTAKE_VOLTAGE)
    }
  }

  fun intakeVoltage() {
    motor.setVoltage(UndertakerConstants.INTAKE_VOLTAGE)
  }


  fun slowIntake(): Command {
    return this.runOnce {
      motor.setVoltage(UndertakerConstants.SLOW_INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(UndertakerConstants.REVERSE_VOLTAGE)
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
    fun createUndertaker(): Undertaker {
      val motor = createSparkMax(
        "ProtoUndertaker Motor",
        UndertakerConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          UndertakerConstants.GEARING
        ),
        inverted = UndertakerConstants.INVERTED,
        currentLimit = UndertakerConstants.CURRENT_LIM,
        slaveSparks = mapOf(Pair(UndertakerConstants.FOLLOLWER_ID, UndertakerConstants.FOLLOWER_INV)),
        enableBrakeMode = UndertakerConstants.BRAKE_MODE
      )

      return Undertaker(motor)
    }
  }
}
