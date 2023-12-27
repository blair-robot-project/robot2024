package frc.team449.control.differential

import edu.wpi.first.math.controller.DifferentialDriveFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.DriveSubsystem
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.drives.DifferentialConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

/**
 * A differential drive (aka. tank drive).
 * @param leftLeader The lead motor of the left side.
 * @param rightLeader The lead motor of the right side.
 * @param ahrs The gyro that is mounted on the chassis.
 * @param feedForward The differential drive feed forward used for calculating the side voltages.
 * @param makeSidePID Used to make two copies of the same PID controller to control both sides of the robot.
 * @param trackwidth The distance between the two wheel sides of the robot.
 */
open class DifferentialDrive(
  private val leftLeader: WrappedMotor,
  private val rightLeader: WrappedMotor,
  private val ahrs: AHRS,
  private val feedForward: DifferentialDriveFeedforward,
  private val makeSidePID: () -> PIDController,
  private val trackwidth: Double
) : DriveSubsystem, SubsystemBase() {
  init {
    leftLeader.encoder.resetPosition(0.0)
    rightLeader.encoder.resetPosition(0.0)
  }

  /** Kinematics used to convert [ChassisSpeeds] to [DifferentialDriveWheelSpeeds] */
  val kinematics = DifferentialDriveKinematics(trackwidth)

  /** Pose estimator that estimates the robot's position as a [Pose2d]. */
  val poseEstimator = DifferentialDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    leftLeader.position,
    rightLeader.position,
    Pose2d()
  )

  /** Velocity PID controller for left side. */
  private val leftPID = makeSidePID()

  /** Velocity PID controller for right side. */
  private val rightPID = makeSidePID()

  var desiredWheelSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

  private var previousTime = Double.NaN
  private var prevWheelSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

  private var prevLeftVel = 0.0
  private var prevRightVel = 0.0
  private var leftVel = 0.0
  private var rightVel = 0.0

  /** Calculate left and right side speeds from given [ChassisSpeeds]. */
  override fun set(desiredSpeeds: ChassisSpeeds) {
    prevWheelSpeeds = desiredWheelSpeeds

    prevLeftVel = desiredWheelSpeeds.leftMetersPerSecond
    prevRightVel = desiredWheelSpeeds.rightMetersPerSecond

    desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
    desiredWheelSpeeds.desaturate(RobotConstants.MAX_LINEAR_SPEED)

    leftVel = desiredWheelSpeeds.leftMetersPerSecond
    rightVel = desiredWheelSpeeds.rightMetersPerSecond
  }

  /** The (x, y, theta) position of the robot on the field. */
  override var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(pose) {
      leftLeader.encoder.resetPosition(0.0)
      rightLeader.encoder.resetPosition(0.0)
      this.poseEstimator.resetPosition(ahrs.heading, leftLeader.position, rightLeader.position, pose)
    }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    val currentTime = Timer.getFPGATimestamp()

    val dt = if (previousTime.isNaN()) 0.02 else currentTime - previousTime

    leftPID.setpoint = leftVel
    rightPID.setpoint = rightVel

    /** Calculates the individual side voltages using a [DifferentialDriveFeedforward]. */
    val sideVoltages = feedForward.calculate(
      prevLeftVel,
      leftVel,
      prevRightVel,
      rightVel,
      dt
    )

    leftLeader.setVoltage(sideVoltages.left + leftPID.calculate(prevLeftVel))

    rightLeader.setVoltage(sideVoltages.right + rightPID.calculate(prevRightVel))

    this.poseEstimator.update(ahrs.heading, this.leftLeader.position, this.rightLeader.position)

    previousTime = currentTime
  }

  companion object {
    /** Create a [DifferentialDrive] using [DifferentialConstants]. */
    private fun makeSide(
      name: String,
      motorId: Int,
      inverted: Boolean,
      encInverted: Boolean,
      wpiEnc: edu.wpi.first.wpilibj.Encoder,
      followers: Map<Int, Boolean>
    ) =
      createSparkMax(
        name = name + "_Side",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator = QuadEncoder.creator(
          wpiEnc,
          DifferentialConstants.DRIVE_EXT_ENC_CPR,
          DifferentialConstants.DRIVE_UPR,
          DifferentialConstants.DRIVE_GEARING,
          encInverted
        ),

        slaveSparks = followers,
        currentLimit = DifferentialConstants.DRIVE_CURRENT_LIM
      )

    fun createDifferentialDrive(ahrs: AHRS): DifferentialDrive {
      return DifferentialDrive(
        leftLeader = makeSide(
          "Left",
          DifferentialConstants.DRIVE_MOTOR_L,
          inverted = false,
          encInverted = false,
          wpiEnc = DifferentialConstants.DRIVE_ENC_LEFT,
          followers = mapOf(
            DifferentialConstants.DRIVE_MOTOR_L1 to false,
            DifferentialConstants.DRIVE_MOTOR_L2 to false
          )
        ),
        rightLeader = makeSide(
          "Right",
          DifferentialConstants.DRIVE_MOTOR_R,
          inverted = true,
          encInverted = true,
          wpiEnc = DifferentialConstants.DRIVE_ENC_RIGHT,
          followers = mapOf(
            DifferentialConstants.DRIVE_MOTOR_R1 to false,
            DifferentialConstants.DRIVE_MOTOR_R2 to false
          )
        ),
        ahrs,
        DifferentialConstants.DRIVE_FEED_FORWARD,
        {
          PIDController(
            DifferentialConstants.DRIVE_KP,
            DifferentialConstants.DRIVE_KI,
            DifferentialConstants.DRIVE_KD
          )
        },
        DifferentialConstants.TRACK_WIDTH
      )
    }

//    fun simOf(
//      drive: DifferentialDrive,
//      kV: Double,
//      kA: Double,
//      angleKV: Double,
//      angleKA: Double,
//      wheelRadius: Double
//    ): DifferentialSim {
//      val drivePlant = LinearSystemId.identifyDrivetrainSystem(
//        kV,
//        kA,
//        angleKV,
//        angleKA
//      )
//      val driveSim = DifferentialDrivetrainSim(
//        drivePlant,
//        DCMotor.getNEO(3),
//        DifferentialConstants.DRIVE_GEARING,
//        drive.trackwidth,
//        wheelRadius,
//        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
//      )
//      return DifferentialSim(driveSim, drive.leftLeader, drive.rightLeader, drive.ahrs, drive.feedforward, drive.makeVelPID, drive.trackwidth)
//    }
  }
}
