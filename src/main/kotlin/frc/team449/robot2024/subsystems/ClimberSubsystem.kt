package frc.team449.robot2024.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.system.motor.WrappedMotor

class ClimberSubsystem(
  val leftMotor: WrappedMotor,
  val rightMotor: WrappedMotor,
  val leftController: ProfiledPIDController,
  val rightController: ProfiledPIDController
) : SubsystemBase() {

}