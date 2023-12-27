package frc.team449.system.motor
//
// import com.ctre.phoenix.motorcontrol.*
// import com.ctre.phoenix.motorcontrol.can.BaseTalon
// import com.ctre.phoenix.motorcontrol.can.VictorSPX
// import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod
// import edu.wpi.first.wpilibj.motorcontrol.MotorController
// import frc.team449.system.encoder.EncoderCreator
//
// /** Wrap a TalonSRX or TalonFX in a WrappedMotor object
// * @see configureSlaveTalon
// * @see createSlaveVictor
// */
// fun <T> createTalon(
//  name: String,
//  motor: T,
//  encCreator: EncoderCreator<T>,
//  enableBrakeMode: Boolean = true,
//  inverted: Boolean = false,
//  currentLimit: Int = 0,
//  enableVoltageComp: Boolean = true,
//  voltageCompSamples: Int = 32,
//  slaveTalons: List<BaseTalon> = listOf(),
//  slaveVictors: List<VictorSPX> = listOf(),
//  reverseSensor: Boolean = false,
//  controlFrameRatesMillis: Map<ControlFrame, Int> = mapOf(),
//  statusFrameRatesMillis: Map<StatusFrameEnhanced, Int> = mapOf(),
//  feedbackDevice: FeedbackDevice? = null
// ): WrappedMotor where T : MotorController, T : BaseTalon {
//  val enc = encCreator.create(name + "Enc", motor, inverted)
//  motor.setInverted(inverted)
//  // Set brake mode
//  val idleMode = if (enableBrakeMode) NeutralMode.Brake else NeutralMode.Coast
//  motor.setNeutralMode(idleMode)
//  for ((frame, period) in controlFrameRatesMillis) {
//    motor.setControlFramePeriod(frame, period)
//  }
//  for ((frame, period) in statusFrameRatesMillis) {
//    motor.setStatusFramePeriod(frame, period)
//  }
//
//  // Setup feedback device if it exists
//  if (feedbackDevice != null) {
//    // CTRE encoder use RPM instead of native units, and can be used as
//    // QuadEncoders, so we switch
//    // them to avoid having to support RPM.
//    if (feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Absolute ||
//      feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Relative
//    ) {
//      motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
//    } else {
//      motor.configSelectedFeedbackSensor(feedbackDevice, 0, 0)
//    }
//    motor.setSensorPhase(reverseSensor)
//  } else {
//    motor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0)
//  }
//
//  // Set the current limit if it was given
//  if (currentLimit > 0) {
//    motor.configSupplyCurrentLimit(
//      SupplyCurrentLimitConfiguration(true, currentLimit.toDouble(), 0.0, 0.0),
//      0
//    )
//  } else {
//    // If we don't have a current limit, disable current limiting.
//    motor.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(), 0)
//  }
//
//  // Enable or disable voltage comp
//  if (enableVoltageComp) {
//    motor.enableVoltageCompensation(true)
//    motor.configVoltageCompSaturation(12.0, 0)
//  }
//  motor.configVoltageMeasurementFilter(voltageCompSamples, 0)
//
//  // Use slot 0
//  motor.selectProfileSlot(0, 0)
//  val port = motor.deviceID
//  // Set up slaves.
//  for (slave in slaveTalons) {
//    setMasterForTalon(
//      slave,
//      port,
//      enableBrakeMode,
//      currentLimit,
//      if (enableVoltageComp) voltageCompSamples else null
//    )
//  }
//  for (slave in slaveVictors) {
//    setMasterForVictor(
//      slave,
//      motor,
//      enableBrakeMode,
//      if (enableVoltageComp) voltageCompSamples else null
//    )
//  }
//  motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms)
//  motor.configVelocityMeasurementWindow(10)
//
//  // Set max voltage
//  motor.configPeakOutputForward(1.0, 0)
//  motor.configPeakOutputReverse(1.0, 0)
//
//  motor.configNominalOutputForward(0.0, 0)
//  motor.configNominalOutputReverse(0.0, 0)
//
//  return WrappedMotor(name, motor, enc)
// }
//
// /** @param slaveTalon Takes a slave TalonSRX or TalonFX and configures it to act as a slave
// * @return The same Talon
// */
// fun configureSlaveTalon(
//  slaveTalon: BaseTalon,
//  invertType: InvertType
// ): BaseTalon {
//  // Turn off features we don't want a slave to have
//  slaveTalon.setInverted(invertType)
//  slaveTalon.configForwardLimitSwitchSource(
//    LimitSwitchSource.Deactivated,
//    LimitSwitchNormal.Disabled,
//    0
//  )
//  slaveTalon.configReverseLimitSwitchSource(
//    LimitSwitchSource.Deactivated,
//    LimitSwitchNormal.Disabled,
//    0
//  )
//  slaveTalon.configForwardSoftLimitEnable(false, 0)
//  slaveTalon.configReverseSoftLimitEnable(false, 0)
//  slaveTalon.configPeakOutputForward(1.0, 0)
//  slaveTalon.enableVoltageCompensation(true)
//  slaveTalon.configVoltageCompSaturation(12.0, 0)
//  slaveTalon.configVoltageMeasurementFilter(32, 0)
//
//  // Slow down frames so we don't overload the CAN bus
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, 0)
//  slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 100, 0)
//
//  return slaveTalon
// }
//
// /**
// * Creates a VictorSPX that will follow some Talon
// *
// * @param port The CAN ID of this Victor SPX.
// * @param invertType Whether to invert this relative to the master. Defaults to not inverting
// * relative to master.
// */
// fun createSlaveVictor(port: Int, invertType: InvertType): VictorSPX {
//  val victorSPX = VictorSPX(port)
//  victorSPX.setInverted(invertType)
//  victorSPX.configPeakOutputForward(1.0, 0)
//  victorSPX.configPeakOutputReverse(-1.0, 0)
//  victorSPX.enableVoltageCompensation(true)
//  victorSPX.configVoltageCompSaturation(12.0, 0)
//  victorSPX.configVoltageMeasurementFilter(32, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_1_General, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_6_Misc, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 100, 0)
//  victorSPX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 100, 0)
//
//  return victorSPX
// }
//
// /**
// * Set this Talon to follow another CAN device.
// *
// * @param port The CAN ID of the device to follow.
// * @param brakeMode Whether this Talon should be in brake mode or coast mode.
// * @param currentLimit The current limit for this Talon. Can be null for no current limit.
// * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
// * compensate voltage.
// */
// private fun setMasterForTalon(
//  slaveTalon: BaseTalon,
//  port: Int,
//  brakeMode: Boolean,
//  currentLimit: Int,
//  voltageCompSamples: Int?
// ) {
//  // Brake mode doesn't automatically follow master
//  slaveTalon.setNeutralMode(if (brakeMode) NeutralMode.Brake else NeutralMode.Coast)
//
//  // Current limiting might not automatically follow master, set it just to be
//  // safe
//  if (currentLimit > 0) {
//    slaveTalon.configSupplyCurrentLimit(
//      SupplyCurrentLimitConfiguration(true, currentLimit.toDouble(), 0.0, 0.0),
//      0
//    )
//  } else {
//    // If we don't have a current limit, disable current limiting.
//    slaveTalon.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(), 0)
//  }
//
//  // Voltage comp might not follow master either
//  if (voltageCompSamples != null) {
//    slaveTalon.enableVoltageCompensation(true)
//    slaveTalon.configVoltageCompSaturation(12.0, 0)
//    slaveTalon.configVoltageMeasurementFilter(voltageCompSamples, 0)
//  } else {
//    slaveTalon.enableVoltageCompensation(false)
//  }
//
//  // Follow the leader
//  slaveTalon[ControlMode.Follower] = port.toDouble()
// }
//
// /**
// * Set this Victor to follow another CAN device.
// *
// * @param toFollow The motor controller to follow.
// * @param brakeMode Whether this Talon should be in brake mode or coast mode.
// * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
// * compensate voltage.
// */
// private fun setMasterForVictor(
//  victorSPX: VictorSPX,
//  toFollow: IMotorController,
//  brakeMode: Boolean,
//  voltageCompSamples: Int?
// ) {
//  // Brake mode doesn't automatically follow master
//  victorSPX.setNeutralMode(if (brakeMode) NeutralMode.Brake else NeutralMode.Coast)
//
//  // Voltage comp might not follow master either
//  if (voltageCompSamples != null) {
//    victorSPX.enableVoltageCompensation(true)
//    victorSPX.configVoltageCompSaturation(12.0, 0)
//    victorSPX.configVoltageMeasurementFilter(voltageCompSamples, 0)
//  } else {
//    victorSPX.enableVoltageCompensation(false)
//  }
//
//  // Follow the leader
//  victorSPX.follow(toFollow)
// }
