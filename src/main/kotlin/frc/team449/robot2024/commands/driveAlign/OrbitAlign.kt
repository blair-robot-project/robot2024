package frc.team449.robot2024.commands.driveAlign

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.auto.AutoConstants
import frc.team449.robot2024.constants.drives.SwerveConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

/**
 * @param drive The holonomic drive you want to align with
 * @param point The point in 2d space you want the drivetrain to face towards
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 */
class OrbitAlign(
  private val drive: SwerveDrive,
  private val controller: XboxController,
  private val point: Translation2d,
  private val headingPID: PIDController = PIDController(
    AutoConstants.ORBIT_KP,
    0.0,
    0.0
  ),
  tolerance: Double = 0.015
) : Command() {

  init {
    addRequirements(drive)
    headingPID.enableContinuousInput(-Math.PI, Math.PI)
    headingPID.setTolerance(tolerance)
  }

  private var fieldToRobot = Translation2d()
  private var robotToPoint = Translation2d()

  private var prevX = 0.0
  private var prevY = 0.0

  private var prevTime = 0.0

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private val directionCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) -1.0 else 1.0 }

  private val timer = Timer()

  private val rotCtrl = RobotConstants.ORTHOGONAL_CONTROLLER

  private var skewConstant = SwerveConstants.SKEW_CONSTANT

  init {
    addRequirements(drive)
    rotCtrl.enableContinuousInput(-PI, PI)
    rotCtrl.setTolerance(0.025)
  }

  override fun initialize() {
    timer.restart()

    prevX = drive.currentSpeeds.vxMetersPerSecond
    prevY = drive.currentSpeeds.vyMetersPerSecond
    prevTime = 0.0
    dx = 0.0
    dy = 0.0
    magAcc = 0.0
    dt = 0.0
    magAccClamped = 0.0
  }

  override fun execute() {
    fieldToRobot = drive.pose.translation
    robotToPoint = point - fieldToRobot
    headingPID.setpoint = robotToPoint.angle.radians

    val currTime = timer.get()
    dt = currTime - prevTime
    prevTime = currTime

    val ctrlX = if (abs(controller.leftY) < RobotConstants.TRANSLATION_DEADBAND) .0 else -controller.leftY
    val ctrlY = if (abs(controller.leftX) < RobotConstants.TRANSLATION_DEADBAND) .0 else -controller.leftX

    val ctrlRadius = sqrt(ctrlX.pow(2) + ctrlY.pow(2)).pow(SwerveConstants.JOYSTICK_FILTER_ORDER)

    val ctrlTheta = atan2(ctrlY, ctrlX)

    val xScaled = ctrlRadius * cos(ctrlTheta) * RobotConstants.MAX_LINEAR_SPEED
    val yScaled = ctrlRadius * sin(ctrlTheta) * RobotConstants.MAX_LINEAR_SPEED

    dx = xScaled - prevX
    dy = yScaled - prevY
    magAcc = hypot(dx / dt, dy / dt)
    magAccClamped = MathUtil.clamp(magAcc, -RobotConstants.MAX_ACCEL, RobotConstants.MAX_ACCEL)

    val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
    val dxClamped = dx * factor
    val dyClamped = dy * factor
    val xClamped = prevX + dxClamped
    val yClamped = prevY + dyClamped

    prevX = xClamped
    prevY = yClamped

    val vel = Translation2d(xClamped, yClamped)

    val rotSpeed = MathUtil.clamp(headingPID.calculate(drive.heading.radians), -RobotConstants.MAX_ROT_SPEED, RobotConstants.MAX_ROT_SPEED)

    /** Quick fix for the velocity skewing towards the direction of rotation
     * by rotating it with offset proportional to how much we are rotating
     **/
    vel.rotateBy(Rotation2d(-rotSpeed * dt * skewConstant))

    val desVel = ChassisSpeeds.fromFieldRelativeSpeeds(
      vel.x * directionCompensation.invoke(),
      vel.y * directionCompensation.invoke(),
      rotSpeed,
      drive.heading
    )

    drive.set(
      desVel
    )
  }

  fun atSetpoint(): Boolean {
    return headingPID.atSetpoint()
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
