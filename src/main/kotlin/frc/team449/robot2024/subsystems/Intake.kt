package frc.team449.robot2024.subsystems

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Intake : Subsystem {

  var motor1 : WrappedMotor = createSparkMax("m1",
    7,
    NEOEncoder.creator(
    1.0,
    1.0
  ))

  var motor2 : WrappedMotor = createSparkMax("m2",
    55,
    NEOEncoder.creator(
    1.0,
    1.0
  ))

  fun driveMotors() {
    motor1.setVoltage(6.0)
    motor2.setVoltage(6.0)
  }

  fun stopMotors() {
    motor1.setVoltage(0.0)
    motor2.setVoltage(0.0)
  }
}
