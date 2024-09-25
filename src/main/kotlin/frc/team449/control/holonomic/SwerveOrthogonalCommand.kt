package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.drives.SwerveConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

class SwerveOrthogonalCommand(
  private val drive: SwerveDrive,
  private val controller: XboxController,
  private val fieldOriented: () -> Boolean = { true }
) : Command() {

  private var prevX = 0.0
  private var prevY = 0.0

  private var prevTime = 0.0

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private var rotScaled = 0.0
  private val allianceCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) PI else 0.0 }
  private val directionCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) -1.0 else 1.0 }

  var atGoal = true

  private var rotRamp = SlewRateLimiter(RobotConstants.ROT_RATE_LIMIT)

  private val timer = Timer()

  private val rotCtrl = PIDController(
    RobotConstants.SNAP_KP,
    RobotConstants.SNAP_KI,
    RobotConstants.SNAP_KD
  )

  private var skewConstant = SwerveConstants.SKEW_CONSTANT

  private var desiredVel = doubleArrayOf(0.0, 0.0, 0.0)

  private var angVelOffset = 0.0
  private var usingAngVelOffset = false

  init {
    addRequirements(drive)
    rotCtrl.enableContinuousInput(-PI, PI)
    rotCtrl.setTolerance(RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD)
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

    rotRamp = SlewRateLimiter(
      RobotConstants.ROT_RATE_LIMIT,
      RobotConstants.NEG_ROT_RATE_LIM,
      drive.currentSpeeds.omegaRadiansPerSecond
    )

    atGoal = true
  }

  fun orthogonalAngle(angle: Double) {
    val desAngle = MathUtil.angleModulus(angle + allianceCompensation.invoke())
    if (abs(desAngle - drive.heading.radians) > RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD &&
      abs(desAngle - drive.heading.radians) < 2 * PI - RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD
    ) {
      rotCtrl.calculate(drive.heading.radians, desAngle)
      atGoal = rotCtrl.atSetpoint()
    }
  }

  fun snapToAngle(angle: Double, angVel: Double = 0.0) {
    val desAngle = MathUtil.angleModulus(angle + allianceCompensation.invoke())
    rotCtrl.calculate(drive.heading.radians, desAngle)
    atGoal = rotCtrl.atSetpoint()
    angVelOffset = angVel
    usingAngVelOffset = true
  }

  fun checkSnapToAngleTolerance(): Boolean {
    return abs(rotCtrl.positionError) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD
  }

  override fun execute() {
    val currTime = timer.get()
    dt = currTime - prevTime
    prevTime = currTime

    val ctrlX = -controller.leftY
    val ctrlY = -controller.leftX

    val ctrlRadius = MathUtil.applyDeadband(
      min(sqrt(ctrlX.pow(2) + ctrlY.pow(2)), 1.0),
      RobotConstants.DRIVE_RADIUS_DEADBAND,
      1.0
    ).pow(SwerveConstants.JOYSTICK_FILTER_ORDER)

    val ctrlTheta = atan2(ctrlY, ctrlX)

    val xScaled = ctrlRadius * cos(ctrlTheta) * drive.maxLinearSpeed
    val yScaled = ctrlRadius * sin(ctrlTheta) * drive.maxLinearSpeed

    var xClamped = xScaled
    var yClamped = yScaled

    if (RobotConstants.USE_ACCEL_LIMIT) {
      dx = xScaled - prevX
      dy = yScaled - prevY
      magAcc = hypot(dx / dt, dy / dt)
      magAccClamped = MathUtil.clamp(magAcc, -RobotConstants.MAX_ACCEL, RobotConstants.MAX_ACCEL)

      val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
      val dxClamped = dx * factor
      val dyClamped = dy * factor
      xClamped = prevX + dxClamped
      yClamped = prevY + dyClamped
    }

    prevX = xClamped
    prevY = yClamped

//    if (controller.bButtonPressed) {
//      snapToAngle(-PI / 3)
//    } else if (controller.xButtonPressed) {
//      snapToAngle(PI / 3)
//    } else if (controller.aButtonPressed) {
//      snapToAngle(0.0)
//    }
//
//    if (controller.aButtonPressed) {
//      orthogonalAngle(0.0)
//    }

    if (atGoal) {
      rotScaled = rotRamp.calculate(
        MathUtil.applyDeadband(
          abs(controller.rightX).pow(SwerveConstants.ROT_FILTER_ORDER),
          RobotConstants.ROTATION_DEADBAND,
          1.0
        ) * -sign(controller.rightX) * drive.maxRotSpeed
      )
    } else {
      if (usingAngVelOffset) {
        rotScaled = MathUtil.clamp(
          rotCtrl.calculate(drive.heading.radians) + angVelOffset,
          -RobotConstants.ALIGN_ROT_SPEED,
          RobotConstants.ALIGN_ROT_SPEED
        )
        atGoal = rotCtrl.atSetpoint()
      } else {
        rotScaled = MathUtil.clamp(
          rotCtrl.calculate(drive.heading.radians),
          -RobotConstants.ALIGN_ROT_SPEED,
          RobotConstants.ALIGN_ROT_SPEED
        )
        atGoal = rotCtrl.atSetpoint()
      }
    }

    usingAngVelOffset = false

    val vel = Translation2d(xClamped, yClamped)

    if (fieldOriented.invoke()) {
      /** Quick fix for the velocity skewing towards the direction of rotation
       * by rotating it with offset proportional to how much we are rotating
       **/
      vel.rotateBy(Rotation2d(-rotScaled * dt * skewConstant))

      val desVel = ChassisSpeeds.fromFieldRelativeSpeeds(
        vel.x * directionCompensation.invoke(),
        vel.y * directionCompensation.invoke(),
        rotScaled,
        drive.heading
      )
      drive.set(
        desVel
      )

      desiredVel[0] = desVel.vxMetersPerSecond
      desiredVel[1] = desVel.vyMetersPerSecond
      desiredVel[2] = desVel.omegaRadiansPerSecond
    } else {
      drive.set(
        ChassisSpeeds(
          vel.x,
          vel.y,
          rotScaled
        )
      )
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Controller X and Y Values")
    builder.addDoubleProperty("1.1 currX", { if (abs(controller.leftY) < RobotConstants.DRIVE_RADIUS_DEADBAND) .0 else -controller.leftY }, null)
    builder.addDoubleProperty("1.2 currY", { if (abs(controller.leftX) < RobotConstants.DRIVE_RADIUS_DEADBAND) .0 else -controller.leftX }, null)
    builder.addDoubleProperty("1.3 prevX", { prevX }, null)
    builder.addDoubleProperty("1.4 prevY", { prevY }, null)

    builder.publishConstString("2.0", "Delta X, Y, Time over one loop")
    builder.addDoubleProperty("2.1 dx", { dx }, null)
    builder.addDoubleProperty("2.2 dy", { dy }, null)
    builder.addDoubleProperty("2.3 dt", { dt }, null)

    builder.publishConstString("3.0", "Magnitude of Acceleration")
    builder.addDoubleProperty("3.1 magAcc", { magAcc }, null)
    builder.addDoubleProperty("3.2 magAccClamped", { magAccClamped }, null)

    builder.publishConstString("4.0", "Turning Skew")
    builder.addDoubleProperty("4.1 skew constant", { skewConstant }, { k: Double -> skewConstant = k })

    builder.publishConstString("5.0", "Given Speeds")
    builder.addDoubleArrayProperty("Chassis Speed", { desiredVel }, null)

    builder.publishConstString("6.0", "Rotation Controller")
    builder.addBooleanProperty("6.1 Controller At Goal", { atGoal }, null)
  }
}
