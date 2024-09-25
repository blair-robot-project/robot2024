package frc.team449.control.holonomic

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.control.vision.VisionSubsystem
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.vision.VisionConstants
import frc.team449.system.AHRS
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.random.Random

class SwerveSimKraken(
  modules: List<SwerveModuleKraken>,
  ahrs: AHRS,
  maxLinearSpeed: Double,
  maxRotSpeed: Double,
  cameras: List<VisionSubsystem>,
  field: Field2d
) : SwerveDriveKraken(modules, ahrs, maxLinearSpeed, maxRotSpeed, cameras, field) {

  private var lastTime = getFPGATimestamp()
  var odoPose = Pose2d()
  var currHeading = Rotation2d()

  private val odometry = SwerveDriveOdometry(
    kinematics,
    currHeading,
    getPositions(),
    RobotConstants.INITIAL_POSE
  )

  /** The (x, y, theta) position of the robot on the field. */
  override var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(value) {
      this.poseEstimator.resetPosition(
        currHeading,
        getPositions(),
        value
      )

      odometry.resetPosition(
        currHeading,
        getPositions(),
        value
      )
    }

  fun resetPos() {
    val newPose = Pose2d(
      odometry.poseMeters.x + Random.nextDouble(-1.0, 1.0),
      odometry.poseMeters.y + Random.nextDouble(-1.0, 1.0),
      odometry.poseMeters.rotation
    )

    odometry.resetPosition(
      currHeading,
      getPositions(),
      newPose
    )
  }

  override fun localize() = try {
    for ((index, camera) in cameras.withIndex()) {
      val result = camera.estimatedPose(Pose2d(pose.x, pose.y, currHeading))
      if (result.isPresent) {
        val presentResult = result.get()
        numTargets[index] = presentResult.targetsUsed.size.toDouble()
        tagDistance[index] = 0.0
        avgAmbiguity[index] = 0.0
        heightError[index] = abs(presentResult.estimatedPose.z)

        for (tag in presentResult.targetsUsed) {
          val tagPose = camera.estimator.fieldTags.getTagPose(tag.fiducialId)
          if (tagPose.isPresent) {
            val estimatedToTag = presentResult.estimatedPose.minus(tagPose.get())
            tagDistance[index] += sqrt(estimatedToTag.x.pow(2) + estimatedToTag.y.pow(2)) / numTargets[index]
            avgAmbiguity[index] = tag.poseAmbiguity / numTargets[index]
          } else {
            tagDistance[index] = Double.MAX_VALUE
            avgAmbiguity[index] = Double.MAX_VALUE
            break
          }
        }

        val estVisionPose = presentResult.estimatedPose.toPose2d()

        visionPose[0 + 3 * index] = estVisionPose.x
        visionPose[1 + 3 * index] = estVisionPose.y
        visionPose[2 + 3 * index] = estVisionPose.rotation.radians

        if (presentResult.timestampSeconds > 0 &&
          avgAmbiguity[index] <= VisionConstants.MAX_AMBIGUITY &&
          numTargets[index] < 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_SINGLE_TAG ||
          numTargets[index] >= 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_MULTI_TAG + (numTargets[index] - 2) * VisionConstants.NUM_TAG_FACTOR &&
          heightError[index] < VisionConstants.MAX_HEIGHT_ERR_METERS
        ) {
          if (enableVisionFusion) {
            poseEstimator.addVisionMeasurement(
              estVisionPose,
              presentResult.timestampSeconds,
              camera.getEstimationStdDevs(numTargets[index].toInt(), tagDistance[index])
            )
          }
          usedVision[index] = true
          usedVisionSights[index] += 1.toLong()
        } else {
          usedVision[index] = false
          rejectedVisionSights[index] += 1.toLong()
        }
      }
    }
  } catch (e: Error) {
    DriverStation.reportError(
      "!!!!!!!!! VISION ERROR !!!!!!!",
      e.stackTrace
    )
  }

  override fun periodic() {
    val currTime = getFPGATimestamp()

    currHeading = currHeading.plus(Rotation2d(super.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime

    set(super.desiredSpeeds)

    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      modules[0].state,
      modules[1].state,
      modules[2].state,
      modules[3].state
    )

    speedMagnitude = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)

    // Update the robot's pose using the gyro heading and the SwerveModulePositions of each module.
    this.poseEstimator.update(
      currHeading,
      getPositions()
    )

    val visionPoseCopy = visionPose.clone()

    if (cameras.isNotEmpty()) localize()

    visionRunning = visionPose[0] != visionPoseCopy[0] ||
      visionPose[1] != visionPoseCopy[1] ||
      visionPose[2] != visionPoseCopy[2]

    // Sets the robot's pose and individual module rotations on the SmartDashboard [Field2d] widget.
    setRobotPose()

    odoPose = odometry.update(
      currHeading,
      getPositions()
    )

    field.getObject("odo").pose = odoPose
  }
}
