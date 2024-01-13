package frc.team449.robot2024

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveOrthogonalCommand
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.system.AHRS
import frc.team449.system.light.Light
import monologue.Annotations
import monologue.Logged

class Robot : RobotBase(), Logged {

  val driveController = XboxController(0)

  val mechController = XboxController(1)

  val ahrs = AHRS(SPI.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  @Annotations.Log.NT
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  @Annotations.Log.NT
  override val drive = SwerveDrive.createSwerve(ahrs, field)

  @Annotations.Log.NT
  override val driveCommand = SwerveOrthogonalCommand(drive, driveController)

  val light = Light.createLight()
//
//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
