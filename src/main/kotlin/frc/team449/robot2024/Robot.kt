package frc.team449.robot2024

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveOrthogonalCommand
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.subsystems.Climber.Companion.createClimber
import frc.team449.robot2024.subsystems.Feeder.Companion.createFeeder
import frc.team449.robot2024.subsystems.Undertaker.Companion.createUndertaker
import frc.team449.robot2024.subsystems.pivot.Pivot.Companion.createPivot
import frc.team449.robot2024.subsystems.shooter.Shooter.Companion.createShooter
import frc.team449.system.AHRS
import frc.team449.system.light.Light.Companion.createLight
import monologue.Annotations.Log
import monologue.Logged

class Robot : RobotBase(), Logged {

  val driveController = CommandXboxController(0)

  val mechController = CommandXboxController(1)

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
  override val driveCommand = SwerveOrthogonalCommand(drive, driveController.hid)

  val light = createLight()

  @Log.NT
  val undertaker = createUndertaker()

  @Log.NT
  val pivot = createPivot(this)

  @Log.NT
  val shooter = createShooter(this)

  @Log.NT
  val feeder = createFeeder()

  val climber = createClimber(this)

  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
