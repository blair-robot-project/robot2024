package frc.team449.robot2024.subsystems
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.subsystem.FeederConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
class Feeder(
  private val motor: WrappedMotor
) : SubsystemBase() {
  fun feed(): Command {
    return this.runOnce {
      motor.setVoltage(FeederConstants.FEEDER_VOLTAGE)
    }
  }

  fun reverse(): Command {
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
    builder.publishConstString("1.0", "Feeder Motor Voltages")
    builder.addDoubleProperty("1.1 Last Feeder Motor Voltage", { motor.lastVoltage }, null)
  }
  companion object {
    fun createFeeder(): Feeder {
      val motor = createSparkMax(
        "Feeder Motor",
        FeederConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = FeederConstants.INVERTED,
        currentLimit = FeederConstants.CURRENT_LIM,
        slaveSparks = mapOf(
          Pair(FeederConstants.FOLLOLWER1_ID, FeederConstants.FOLLOWER1_INV),
          Pair(FeederConstants.FOLLOLWER2_ID, FeederConstants.FOLLOWER2_INV),
          Pair(FeederConstants.FOLLOLWER3_ID, FeederConstants.FOLLOWER3_INV),
        )
      )

      return Feeder(motor)
    }
  }
}