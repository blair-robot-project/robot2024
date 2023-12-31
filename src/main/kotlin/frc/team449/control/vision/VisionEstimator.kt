package frc.team449.control.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.robot2023.constants.vision.VisionConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.OpenCVHelp
import org.photonvision.targeting.PNPResults
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import java.util.Optional
import kotlin.math.PI
import kotlin.math.abs

/**
 * This class uses normal multi-tag PNP and lowest ambiguity using the gyro rotation
 *  for the internal cam-to-tag transform as a fallback strategy
 */
class VisionEstimator(
  private val tagLayout: AprilTagFieldLayout,
  val camera: PhotonCamera,
  private val robotToCam: Transform3d
) : PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, camera, robotToCam) {
  private val reportedErrors: HashSet<Int> = HashSet()
  private var driveHeading: Rotation2d? = null
  private var lastPose: Pose3d? = null

  fun estimatedPose(currPose: Pose2d): Optional<EstimatedRobotPose> {
    driveHeading = currPose.rotation
    lastPose = Pose3d(currPose.x, currPose.y, 0.0, Rotation3d(0.0, 0.0, currPose.rotation.radians))
    return updatePose(camera.latestResult)
  }

  private fun updatePose(cameraResult: PhotonPipelineResult?): Optional<EstimatedRobotPose> {
    // Time in the past -- give up, since the following if expects times > 0
    if (cameraResult!!.timestampSeconds < 0) {
      return Optional.empty()
    }

    // If the pose cache timestamp was set, and the result is from the same timestamp, return an
    // empty result
    if (poseCacheTimestampSeconds > 0 &&
      abs(poseCacheTimestampSeconds - cameraResult.timestampSeconds) < 1e-6
    ) {
      return Optional.empty()
    }

    // Remember the timestamp of the current result used
    poseCacheTimestampSeconds = cameraResult.timestampSeconds

    // If no targets seen, trivial case -- return empty result
    return if (!cameraResult.hasTargets()) {
      Optional.empty()
    } else {
      multiTagPNPStrategy(cameraResult)
    }
  }

  private fun checkBest(check: Pose3d?, opt1: Pose3d?, opt2: Pose3d?): Pose3d? {
    if (check == null || opt1 == null || opt2 == null) return null
    val dist1 = check.translation.toTranslation2d().getDistance(opt1.translation.toTranslation2d())
    val dist2 = check.translation.toTranslation2d().getDistance(opt2.translation.toTranslation2d())

    return if (dist1 < dist2) {
      opt1
    } else {
      opt2
    }
  }

  private fun multiTagPNPStrategy(result: PhotonPipelineResult): Optional<EstimatedRobotPose> {
    // Arrays we need declared up front
    val visCorners = ArrayList<TargetCorner>()
    val knownVisTags = ArrayList<AprilTag>()
    val fieldToCams = ArrayList<Pose3d>()
    val fieldToCamsAlt = ArrayList<Pose3d>()
    val usedTargets = ArrayList<PhotonTrackedTarget>()
    if (result.getTargets().size < 2) {
      // Run fallback strategy instead
      return lowestAmbiguityStrategy(result)
    }

    for (target: PhotonTrackedTarget in result.getTargets()) {
      val tagPoseOpt = tagLayout.getTagPose(target.fiducialId)
      if (tagPoseOpt.isEmpty) {
        reportFiducialPoseError(target.fiducialId)
        continue
      }
      val tagPose = tagPoseOpt.get()

      var unique = true

      for (addedTarget in usedTargets) {
        if (addedTarget.fiducialId == target.fiducialId) unique = false
      }

      if (unique) usedTargets.add(target)

      visCorners.addAll(target.detectedCorners)
      // actual layout poses of visible tags -- not exposed, so have to recreate
      knownVisTags.add(AprilTag(target.fiducialId, tagPose))
      fieldToCams.add(tagPose.transformBy(target.bestCameraToTarget.inverse()))
      fieldToCamsAlt.add(tagPose.transformBy(target.alternateCameraToTarget.inverse()))
    }
    val cameraMatrixOpt = camera.cameraMatrix
    val distCoeffsOpt = camera.distCoeffs
    val hasCalibData = cameraMatrixOpt.isPresent && distCoeffsOpt.isPresent

    // multi-target solvePNP
    if (hasCalibData && visCorners.size == knownVisTags.size * 4 && knownVisTags.isNotEmpty()) {
      val cameraMatrix = cameraMatrixOpt.get()
      val distCoeffs = distCoeffsOpt.get()
      val pnpResults = estimateCamPosePNP(cameraMatrix, distCoeffs, usedTargets.toList())
      val best = Pose3d()
        .plus(pnpResults.best) // field-to-camera
        .plus(robotToCam.inverse()) // field-to-robot

      val alternate = Pose3d()
        .plus(pnpResults.alt)
        .plus(robotToCam.inverse())

      return Optional.of(
        EstimatedRobotPose(
          checkBest(lastPose, best, alternate) ?: best,
          result.timestampSeconds,
          usedTargets,
          PoseStrategy.MULTI_TAG_PNP_ON_RIO
        )
      )
    }

    return Optional.empty()
  }

  /**
   * Return the estimated position of the robot with the lowest position ambiguity from a List of
   * pipeline results.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated timestamp of this
   * estimation.
   */
  private fun lowestAmbiguityStrategy(result: PhotonPipelineResult): Optional<EstimatedRobotPose> {
    var lowestAmbiguityTarget: PhotonTrackedTarget? = null
    var lowestAmbiguityScore = 10.0
    for (target: PhotonTrackedTarget in result.targets) {
      val targetPoseAmbiguity = target.poseAmbiguity
      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1.0 && targetPoseAmbiguity < lowestAmbiguityScore) {
        lowestAmbiguityScore = targetPoseAmbiguity
        lowestAmbiguityTarget = target
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null) return Optional.empty()
    val targetFiducialId = lowestAmbiguityTarget.fiducialId
    val targetPosition = tagLayout.getTagPose(targetFiducialId)
    if (targetPosition.isEmpty) {
      reportFiducialPoseError(targetFiducialId)
      return Optional.empty()
    }

    val bestPose = targetPosition
      .get()
      .transformBy(
        Transform3d(
          lowestAmbiguityTarget.bestCameraToTarget.translation,
          Rotation3d(
            lowestAmbiguityTarget.bestCameraToTarget.rotation.x,
            lowestAmbiguityTarget.bestCameraToTarget.rotation.y,
            driveHeading!!.radians + robotToCam.rotation.z - targetPosition.get().rotation.z
          )
        ).inverse()
      )
      .transformBy(robotToCam.inverse())

    val altPose = targetPosition
      .get()
      .transformBy(
        Transform3d(
          lowestAmbiguityTarget.alternateCameraToTarget.translation,
          Rotation3d(
            lowestAmbiguityTarget.alternateCameraToTarget.rotation.x,
            lowestAmbiguityTarget.alternateCameraToTarget.rotation.y,
            driveHeading!!.radians + robotToCam.rotation.z - targetPosition.get().rotation.z
          )
        ).inverse()
      )
      .transformBy(robotToCam.inverse())

    val usedPose = checkBest(lastPose, bestPose, altPose) ?: bestPose

    if (usedPose == bestPose) {
      val camBestPose = targetPosition
        .get()
        .transformBy(
          Transform3d(
            lowestAmbiguityTarget.bestCameraToTarget.translation,
            lowestAmbiguityTarget.bestCameraToTarget.rotation
          ).inverse()
        )
        .transformBy(robotToCam.inverse())

      if (abs(camBestPose.rotation.z * (180 / (2 * PI)) - driveHeading!!.degrees) > VisionConstants.SINGLE_TAG_HEADING_MAX_DEV_DEG) {
        DriverStation.reportWarning("Best Single Tag Heading over Max Deviation, deviated by ${abs(camBestPose.rotation.z * (180 / (2 * PI)) - driveHeading!!.degrees)}", false)
        return Optional.empty()
      }
    } else {

      val camAltPose = targetPosition
        .get()
        .transformBy(
          Transform3d(
            lowestAmbiguityTarget.alternateCameraToTarget.translation,
            lowestAmbiguityTarget.alternateCameraToTarget.rotation
          ).inverse()
        )
        .transformBy(robotToCam.inverse())

      if (abs(camAltPose.rotation.z * (180 / (2 * PI)) - driveHeading!!.degrees) > VisionConstants.SINGLE_TAG_HEADING_MAX_DEV_DEG) {
        DriverStation.reportWarning("Alt Single Tag Heading over Max Deviation, deviated by ${abs(camAltPose.rotation.z * (180 / (2 * PI)) - driveHeading!!.degrees)}", false)
        return Optional.empty()
      }
    }

    return Optional.of(
      EstimatedRobotPose(
        checkBest(lastPose, bestPose, altPose) ?: bestPose,
        result.timestampSeconds,
        mutableListOf(lowestAmbiguityTarget),
        PoseStrategy.LOWEST_AMBIGUITY
      )
    )
  }

  private fun reportFiducialPoseError(fiducialId: Int) {
    if (!reportedErrors.contains(fiducialId)) {
      DriverStation.reportError(
        "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: $fiducialId",
        false
      )
      reportedErrors.add(fiducialId)
    }
  }

  private fun estimateCamPosePNP(
    cameraMatrix: Matrix<N3?, N3?>?,
    distCoeffs: Matrix<N5?, N1?>?,
    visTags: List<PhotonTrackedTarget>?,
  ): PNPResults {
    if (visTags == null || tagLayout.tags.isEmpty() || visTags.isEmpty()) {
      return PNPResults()
    }
    val corners = java.util.ArrayList<TargetCorner>()
    val knownTags = java.util.ArrayList<AprilTag>()
    // ensure these are AprilTags in our layout
    for (tgt in visTags) {
      val id = tgt.fiducialId
      tagLayout
        .getTagPose(id)
        .ifPresent { pose: Pose3d? ->
          knownTags.add(AprilTag(id, pose))
          corners.addAll(tgt.detectedCorners)
        }
    }
    if (knownTags.size == 0 || corners.size == 0 || corners.size % 4 != 0) {
      return PNPResults()
    }
    val points = OpenCVHelp.cornersToPoints(corners)

    // single-tag pnp
    return if (knownTags.size == 1) {
      val camToTag = OpenCVHelp.solvePNP_SQUARE(cameraMatrix, distCoeffs, VisionConstants.TAG_MODEL.vertices, points)
      if (!camToTag.isPresent) return PNPResults()
      val bestPose = knownTags[0].pose.transformBy(camToTag.best.inverse())
      var altPose: Pose3d? = Pose3d()
      if (camToTag.ambiguity != 0.0) altPose = knownTags[0].pose.transformBy(camToTag.alt.inverse())
      val o = Pose3d()
      PNPResults(
        Transform3d(o, bestPose),
        Transform3d(o, altPose),
        camToTag.ambiguity,
        camToTag.bestReprojErr,
        camToTag.altReprojErr
      )
    } else {
      val objectTrls = java.util.ArrayList<Translation3d>()
      for (tag in knownTags) objectTrls.addAll(VisionConstants.TAG_MODEL.getFieldVertices(tag.pose))
      val camToOrigin = OpenCVHelp.solvePNP_SQPNP(cameraMatrix, distCoeffs, objectTrls, points)
      if (!camToOrigin.isPresent) PNPResults() else PNPResults(
        camToOrigin.best.inverse(),
        camToOrigin.alt.inverse(),
        camToOrigin.ambiguity,
        camToOrigin.bestReprojErr,
        camToOrigin.altReprojErr
      )
    }
  }
}
