package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.XboxController
import frc.team449.robot2024.constants.RobotConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

object RotationCommands {
  private val allianceCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) 0.0 else PI }
  private val directionCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) -1.0 else 1.0 }
  private val rotCtrl = RobotConstants.ORTHOGONAL_CONTROLLER
  private var rotRamp = SlewRateLimiter(RobotConstants.ROT_RATE_LIMIT)

  val regularRotation = { controller: XboxController,
                          drive: HolonomicDrive,
                          atGoal: Boolean ->
    val rotScaled: Double
    if (controller.xButtonPressed) {
      val desAngleA = MathUtil.angleModulus(7 * PI / 4 + allianceCompensation.invoke())
      if (abs(desAngleA - drive.heading.radians) > 0.075 && abs(desAngleA - drive.heading.radians) < 2 * PI - 0.075) {
        rotCtrl.setpoint = desAngleA
      }
    } else if (controller.yButtonPressed) {
      val desAngleY = MathUtil.angleModulus(3 * PI / 2 + allianceCompensation.invoke())
      if (abs(desAngleY - drive.heading.radians) > 0.075 && abs(desAngleY - drive.heading.radians) < 2 * PI - 0.075) {
        rotCtrl.setpoint = desAngleY
      }
    }
    if (atGoal) {
      rotScaled = rotRamp.calculate(
        (if (abs(controller.rightX) < RobotConstants.ROTATION_DEADBAND) .0 else -controller.rightX) *
          drive.maxRotSpeed
      )
    } else {
      rotScaled = MathUtil.clamp(
        rotCtrl.calculate(drive.heading.radians),
        -RobotConstants.ALIGN_ROT_SPEED,
        RobotConstants.ALIGN_ROT_SPEED
      )
    }
    rotScaled
  }

}
