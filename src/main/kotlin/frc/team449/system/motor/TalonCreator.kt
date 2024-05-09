package frc.team449.system.motor



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
