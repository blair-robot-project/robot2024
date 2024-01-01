package frc.team449.control.holonomic

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.vision.VisionSubsystem
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.drives.MecanumConstants
import frc.team449.robot2024.constants.vision.VisionConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

/**
 * @param frontLeftMotor the front left motor
 * @param frontRightMotor the front right motor
 * @param backLeftMotor the back left motor
 * @param backRightMotor the back right motor
 * @param frontLeftLocation the offset of the front left wheel to the center of the robot
 * @param frontRightLocation the offset of the front right wheel to the center of the robot
 * @param backLeftLocation the offset of the back left wheel to the center of the robot
 * @param backRightLocation the offset of the back right wheel to the center of the robot
 * @param maxLinearSpeed the maximum translation speed of the chassis.
 * @param maxRotSpeed the maximum rotation speed of the chassis
 * @param feedForward the SimpleMotorFeedforward for mecanum
 * @param controller the PIDController for the robot
 */
open class MecanumDrive(
  private val frontLeftMotor: WrappedMotor,
  private val frontRightMotor: WrappedMotor,
  private val backLeftMotor: WrappedMotor,
  private val backRightMotor: WrappedMotor,
  frontLeftLocation: Translation2d,
  frontRightLocation: Translation2d,
  backLeftLocation: Translation2d,
  backRightLocation: Translation2d,
  private val ahrs: AHRS,
  override var maxLinearSpeed: Double,
  override var maxRotSpeed: Double,
  private val feedForward: SimpleMotorFeedforward,
  private val controller: () -> PIDController,
  private val cameras: List<VisionSubsystem> = mutableListOf()
) : HolonomicDrive, SubsystemBase() {

  private val flController = controller()
  private val frController = controller()
  private val blController = controller()
  private val brController = controller()

  private var lastTime = Timer.getFPGATimestamp()

  val kinematics = MecanumDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
  )

  private val poseEstimator = MecanumDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    RobotConstants.INITIAL_POSE,
    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .0005), // [x, y, theta] other estimates
    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .0005) // [x, y, theta] vision estimates
  )

  override var pose: Pose2d
    get() {
      return this.poseEstimator.estimatedPosition
    }
    set(value) {
      this.poseEstimator.resetPosition(ahrs.heading, getPositions(), value)
    }

  private var desiredWheelSpeeds = MecanumDriveWheelSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
    desiredWheelSpeeds.desaturate(MecanumConstants.MAX_ATTAINABLE_WHEEL_SPEED)
  }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    val currTime = Timer.getFPGATimestamp()

    val frontLeftPID = flController.calculate(frontLeftMotor.velocity, desiredWheelSpeeds.frontLeftMetersPerSecond)
    val frontRightPID = frController.calculate(frontRightMotor.velocity, desiredWheelSpeeds.frontRightMetersPerSecond)
    val backLeftPID = blController.calculate(backLeftMotor.velocity, desiredWheelSpeeds.rearLeftMetersPerSecond)
    val backRightPID = brController.calculate(backRightMotor.velocity, desiredWheelSpeeds.rearRightMetersPerSecond)

    val frontLeftFF = feedForward.calculate(
      desiredWheelSpeeds.frontLeftMetersPerSecond
    )
    val frontRightFF = feedForward.calculate(
      desiredWheelSpeeds.frontRightMetersPerSecond
    )
    val backLeftFF = feedForward.calculate(
      desiredWheelSpeeds.rearLeftMetersPerSecond
    )
    val backRightFF = feedForward.calculate(
      desiredWheelSpeeds.rearRightMetersPerSecond
    )

    frontLeftMotor.setVoltage(frontLeftPID + frontLeftFF)
    frontRightMotor.setVoltage(frontRightPID + frontRightFF)
    backLeftMotor.setVoltage(backLeftPID + backLeftFF)
    backRightMotor.setVoltage(backRightPID + backRightFF)

    if (cameras.isNotEmpty()) localize()

    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    lastTime = currTime
  }

  /**
   * @return the position readings of the wheels bundled into one object (meters)
   */
  private fun getPositions(): MecanumDriveWheelPositions =
    MecanumDriveWheelPositions(
      frontLeftMotor.position,
      frontRightMotor.position,
      backLeftMotor.position,
      backRightMotor.position
    )

  /**
   * @return the velocity readings of the wheels bundled into one object (meters/s)
   */
  private fun getSpeeds(): MecanumDriveWheelSpeeds =
    MecanumDriveWheelSpeeds(
      frontLeftMotor.velocity,
      frontRightMotor.velocity,
      backLeftMotor.velocity,
      backRightMotor.velocity
    )

  private fun localize() {
    for (camera in cameras) {
      val result = camera.estimatedPose(Pose2d(pose.x, pose.y, ahrs.heading))
      if (result.isPresent) {
        poseEstimator.addVisionMeasurement(
          result.get().estimatedPose.toPose2d(),
          result.get().timestampSeconds
        )
      }
    }
  }

  companion object {

    /** Helper method to create a motor for each wheel */
    private fun createCorner(name: String, motorID: Int, inverted: Boolean): WrappedMotor {
      return createSparkMax(
        name,
        motorID,
        NEOEncoder.creator(
          MecanumConstants.DRIVE_UPR,
          MecanumConstants.DRIVE_GEARING
        ),
        inverted = inverted,
        currentLimit = MecanumConstants.CURRENT_LIM
      )
    }

    /** Create a new Mecanum Drive from DriveConstants */
    fun createMecanum(ahrs: AHRS): MecanumDrive {
      return MecanumDrive(
        createCorner("frontLeft", MecanumConstants.DRIVE_MOTOR_FL, false),
        createCorner("frontRight", MecanumConstants.DRIVE_MOTOR_FR, true),
        createCorner("backLeft", MecanumConstants.DRIVE_MOTOR_BL, false),
        createCorner("backRight", MecanumConstants.DRIVE_MOTOR_BR, true),
        Translation2d(MecanumConstants.WHEELBASE / 2, MecanumConstants.TRACKWIDTH / 2),
        Translation2d(MecanumConstants.WHEELBASE / 2, -MecanumConstants.TRACKWIDTH / 2),
        Translation2d(-MecanumConstants.WHEELBASE / 2, MecanumConstants.TRACKWIDTH / 2),
        Translation2d(-MecanumConstants.WHEELBASE / 2, -MecanumConstants.TRACKWIDTH / 2),
        ahrs,
        RobotConstants.MAX_LINEAR_SPEED,
        RobotConstants.MAX_ROT_SPEED,
        SimpleMotorFeedforward(MecanumConstants.DRIVE_KS, MecanumConstants.DRIVE_KV, MecanumConstants.DRIVE_KA),
        { PIDController(MecanumConstants.DRIVE_KP, MecanumConstants.DRIVE_KI, MecanumConstants.DRIVE_KD) },
        VisionConstants.ESTIMATORS
      )
    }
  }
}
