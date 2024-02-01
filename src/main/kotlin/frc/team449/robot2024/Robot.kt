package frc.team449.robot2024

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveOrthogonalCommand
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.subsystems.Feeder.Companion.createFeeder
import frc.team449.robot2024.subsystems.Undertaker.Companion.createProtoUndertaker
import frc.team449.robot2024.subsystems.pivot.Pivot
import frc.team449.robot2024.subsystems.shooter.Shooter.Companion.createShooter
import frc.team449.robot2024.subsystems.pivot.Pivot.Companion.createShooter
import frc.team449.robot2024.subsystems.shooter.Shooter
import frc.team449.system.AHRS
import frc.team449.system.light.Light
import monologue.Annotations.Log
import monologue.Logged

class Robot : RobotBase(), Logged {

  val driveController = XboxController(0)

  val mechController = XboxController(1)

  val ahrs = AHRS(SPI.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  @Log.NT
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  @Log.NT
  override val drive = SwerveDrive.createSwerve(ahrs, field)

  @Log.NT
  override val driveCommand = SwerveOrthogonalCommand(drive, driveController)

  val light = Light.createLight()

  val undertaker = createProtoUndertaker()
  val feeder = createFeeder()
  val shooter = Shooter.createShooter(this)
  val pivot = Pivot.createShooter(this);
//
//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
