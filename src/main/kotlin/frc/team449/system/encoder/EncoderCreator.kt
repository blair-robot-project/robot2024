package frc.team449.system.encoder

import edu.wpi.first.wpilibj.motorcontrol.MotorController

/**
 * Create an encoder given a motor controller and its configuration
 *
 * @param <M> The type of the motor controller
 */
fun interface EncoderCreator<M : MotorController> {
  fun create(encName: String, motor: M, inverted: Boolean): Encoder
}
