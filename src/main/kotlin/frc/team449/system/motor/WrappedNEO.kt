package frc.team449.system.motor

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.team449.system.encoder.Encoder

/** Our own wrapper grouping the motor controller and encoder for a motor */
class WrappedNEO(
  val motorObj: CANSparkMax,
  val encoder: Encoder
) : MotorController by motorObj {
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
    motorObj.setVoltage(volts)
    this.lastVoltage = volts
  }

  override fun stopMotor() {
    motorObj.stopMotor()
    this.lastVoltage = 0.0
  }

  override fun set(output: Double) {
    motorObj.set(output)
    this.lastVoltage = output * RobotController.getBatteryVoltage()
  }
}
