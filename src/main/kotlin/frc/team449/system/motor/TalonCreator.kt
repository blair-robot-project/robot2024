package frc.team449.system.motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX

/** Wrap a TalonSRX or TalonFX in a WrappedMotor object */
fun createTalon(
  id: Int,
  config: TalonFXConfiguration,
  canbus: String = "",
  followerTalons: List<Pair<Int, Boolean>> = listOf()
): TalonFX {
  val motor = TalonFX(id, canbus)
  config.Voltage.PeakReverseVoltage = -12.0
  config.Voltage.PeakForwardVoltage = 12.0
  config.MotorOutput.DutyCycleNeutralDeadband = 0.001
  motor.configurator.apply(config)
  for (f in followerTalons) {
    val followerTalon = createTalon(f.first, config)
    followerTalon.setControl(Follower(id, f.second))
  }
  return motor
}
