package frc.team449.system.motor

 import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.BaseTalon
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.VoltageConfigs
 import com.ctre.phoenix6.controls.Follower
 import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

/** Wrap a TalonSRX or TalonFX in a WrappedMotor object
 * @see configureSlaveTalon
 * @see createSlaveVictor
 */
 fun createTalon(
  id: Int,
  canbus: String = "",
  enableBrakeMode: NeutralModeValue = NeutralModeValue.Brake,
  inverted: InvertedValue = InvertedValue.Clockwise_Positive,
  deadband: Double = 0.001,
  currentLimit: Double = 0.0,
  followerTalons: List<Pair<Int, InvertedValue>> = listOf()
 ): TalonFX {
   val motor = TalonFX(id, canbus)
   motor.configurator.apply(TalonFXConfiguration())

   val motorConfigs = MotorOutputConfigs()
   motorConfigs.Inverted = inverted
   motorConfigs.NeutralMode = enableBrakeMode
   motorConfigs.DutyCycleNeutralDeadband = deadband
   motor.configurator.apply(motorConfigs)

   val currentConfigs = CurrentLimitsConfigs()
   currentConfigs.StatorCurrentLimit = currentLimit
   currentConfigs.SupplyCurrentLimit = currentLimit
   motor.configurator.apply(currentConfigs)

   val voltageConfigs = VoltageConfigs()
   voltageConfigs.PeakForwardVoltage = 12.0
   voltageConfigs.PeakReverseVoltage = -12.0

   for (f in followerTalons) {
     val followerTalon = createTalon(f.first, inverted = f.second)
     followerTalon.setControl(Follower(id, true))
   }
   return motor
 }

 /**
 * Creates a VictorSPX that will follow some Talon
 *
 * @param port The CAN ID of this Victor SPX.
 * @param invertType Whether to invert this relative to the master. Defaults to not inverting
 * relative to master.
 */
 fun createSlaveVictor(port: Int, invertType: InvertType): VictorSPX {
  val victorSPX = VictorSPX(port)
  victorSPX.setInverted(invertType)
  victorSPX.configPeakOutputForward(1.0, 0)
  victorSPX.configPeakOutputReverse(-1.0, 0)
  victorSPX.enableVoltageCompensation(true)
  victorSPX.configVoltageCompSaturation(12.0, 0)
  victorSPX.configVoltageMeasurementFilter(32, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_1_General, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_6_Misc, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 100, 0)
  victorSPX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 100, 0)

  return victorSPX
 }

 /**
 * Set this Talon to follow another CAN device.
 *
 * @param port The CAN ID of the device to follow.
 * @param brakeMode Whether this Talon should be in brake mode or coast mode.
 * @param currentLimit The current limit for this Talon. Can be null for no current limit.
 * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
 * compensate voltage.
 */
 private fun setMasterForTalon(
  slaveTalon: BaseTalon,
  port: Int,
  brakeMode: Boolean,
  currentLimit: Int,
  voltageCompSamples: Int?
 ) {
  // Brake mode doesn't automatically follow master
  slaveTalon.setNeutralMode(if (brakeMode) NeutralMode.Brake else NeutralMode.Coast)

  // Current limiting might not automatically follow master, set it just to be
  // safe
  if (currentLimit > 0) {
    slaveTalon.configSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration(true, currentLimit.toDouble(), 0.0, 0.0),
      0
    )
  } else {
    // If we don't have a current limit, disable current limiting.
    slaveTalon.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(), 0)
  }

  // Voltage comp might not follow master either
  if (voltageCompSamples != null) {
    slaveTalon.enableVoltageCompensation(true)
    slaveTalon.configVoltageCompSaturation(12.0, 0)
    slaveTalon.configVoltageMeasurementFilter(voltageCompSamples, 0)
  } else {
    slaveTalon.enableVoltageCompensation(false)
  }

  // Follow the leader
  slaveTalon[ControlMode.Follower] = port.toDouble()
 }

 /**
 * Set this Victor to follow another CAN device.
 *
 * @param toFollow The motor controller to follow.
 * @param brakeMode Whether this Talon should be in brake mode or coast mode.
 * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
 * compensate voltage.
 */
 private fun setMasterForVictor(
  victorSPX: VictorSPX,
  toFollow: IMotorController,
  brakeMode: Boolean,
  voltageCompSamples: Int?
 ) {
  // Brake mode doesn't automatically follow master
  victorSPX.setNeutralMode(if (brakeMode) NeutralMode.Brake else NeutralMode.Coast)

  // Voltage comp might not follow master either
  if (voltageCompSamples != null) {
    victorSPX.enableVoltageCompensation(true)
    victorSPX.configVoltageCompSaturation(12.0, 0)
    victorSPX.configVoltageMeasurementFilter(voltageCompSamples, 0)
  } else {
    victorSPX.enableVoltageCompensation(false)
  }

  // Follow the leader
  victorSPX.follow(toFollow)
 }
