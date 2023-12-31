package frc.team449.system.motor

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.REVLibError
import com.revrobotics.SparkMaxLimitSwitch
import edu.wpi.first.wpilibj.RobotController
import frc.team449.system.encoder.EncoderCreator

/**
 * Create a Spark Max with the given configurations
 *
 * @param name The motor's name
 * @param id The motor's CAN ID
 */

// TODO: Test if disabling voltage compensation helps reduce brownouts
fun createSparkMax(
  name: String,
  id: Int,
  encCreator: EncoderCreator<CANSparkMax>,
  enableBrakeMode: Boolean = true,
  inverted: Boolean = false,
  currentLimit: Int = 0,
  enableVoltageComp: Boolean = false,
  slaveSparks: Map<Int, Boolean> = mapOf(),
  controlFrameRateMillis: Int = -1,
  statusFrameRatesMillis: Map<CANSparkMaxLowLevel.PeriodicFrame, Int> = mapOf()
): WrappedMotor {
  val motor = CANSparkMax(
    id,
    CANSparkMaxLowLevel.MotorType.kBrushless
  )
  if (motor.lastError != REVLibError.kOk) {
    println(
      "Motor could not be constructed on port " +
        id +
        " due to error " +
        motor.lastError
    )
  }

  motor.restoreFactoryDefaults()

  val enc = encCreator.create(name + "Enc", motor, inverted)

  val brakeMode =
    if (enableBrakeMode) {
      CANSparkMax.IdleMode.kBrake
    } else {
      CANSparkMax.IdleMode.kCoast
    }

  motor.inverted = inverted
  // Set brake mode
  motor.idleMode = brakeMode

  // Set frame rates
  if (controlFrameRateMillis >= 1) {
    // Must be between 1 and 100 ms.
    motor.setControlFramePeriodMs(controlFrameRateMillis)
  }

  for ((statusFrame, period) in statusFrameRatesMillis) {
    motor.setPeriodicFramePeriod(statusFrame, period)
  }

  // Set the current limit if it was given
  if (currentLimit > 0) {
    motor.setSmartCurrentLimit(currentLimit)
  }

  if (enableVoltageComp) {
    motor.enableVoltageCompensation(RobotController.getBatteryVoltage())
  } else {
    motor.disableVoltageCompensation()
  }

  for ((slavePort, slaveInverted) in slaveSparks) {
    val slave = createFollowerSpark(slavePort)
    slave.restoreFactoryDefaults()
    slave.follow(motor, slaveInverted)
    slave.idleMode = brakeMode
    if (currentLimit > 0) {
      slave.setSmartCurrentLimit(currentLimit)
    }
    slave.burnFlash()
  }

  motor.burnFlash()

  return WrappedMotor(motor, enc, { motor.get() }, { motor.appliedOutput }, { motor.busVoltage }, { motor.outputCurrent })
}

/**
 * Create a Spark that will follow another Spark
 *
 * @param port The follower's CAN ID
 */
private fun createFollowerSpark(port: Int): CANSparkMax {
  val follower = CANSparkMax(
    port,
    CANSparkMaxLowLevel.MotorType.kBrushless
  )

  follower
    .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
    .enableLimitSwitch(false)
  follower
    .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
    .enableLimitSwitch(false)

  follower.setPeriodicFramePeriod(
    CANSparkMaxLowLevel.PeriodicFrame.kStatus0,
    100
  )
  follower.setPeriodicFramePeriod(
    CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
    100
  )
  follower.setPeriodicFramePeriod(
    CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
    100
  )

  return follower
}
