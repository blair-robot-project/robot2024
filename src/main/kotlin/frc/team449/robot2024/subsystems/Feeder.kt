package frc.team449.robot2024.subsystems

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
    }
  }

  fun intakeVoltage() {
    motor.setVoltage(FeederConstants.INTAKE_VOLTAGE)
  }

  fun autoShootIntake(): Command {
    return this.runOnce {
      motor.setVoltage(FeederConstants.AUTO_SHOOT_INTAKE_VOLTAGE)
    }
  }

  fun slowIntake(): Command {
    return this.runOnce {
      motor.setVoltage(FeederConstants.SLOW_INTAKE_VOLTAGE)
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
    fun createFeeder(): Feeder {
      val motor = createSparkMax(
        "Feeder Motor",
        FeederConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          FeederConstants.GEARING
        ),
        inverted = FeederConstants.INVERTED,
        currentLimit = FeederConstants.CURRENT_LIM,
        enableBrakeMode = FeederConstants.BRAKE_MODE
      )

      return Feeder(motor)
    }
  }
}
