package frc.team449.system.motor

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.team449.system.encoder.Encoder
import java.util.function.DoubleSupplier

/** Our own wrapper grouping the motor controller and encoder for a motor */
class WrappedMotor(
  private val motor: MotorController,
  val encoder: Encoder,
  val getDutyCycle: DoubleSupplier? = null,
  val appliedOutput: DoubleSupplier? = null,
  val busVoltage: DoubleSupplier? = null,
  val outputCurrent: DoubleSupplier? = null
) : MotorController by motor {
  /**
   * The last set voltage for this motor (through [setVoltage] or [set])
   */
  var lastVoltage = 0.0
    private set

  /** Position in meters or whatever unit you set */
  val position: Double
    get() = encoder.position

  /** Velocity in meters per second or whatever unit you set */
  val velocity: Double
    get() = encoder.velocity

  override fun setVoltage(volts: Double) {
    motor.setVoltage(volts)
    this.lastVoltage = volts
  }

  override fun set(output: Double) {
    motor.set(output)
    this.lastVoltage = output * RobotController.getBatteryVoltage()
  }
}
