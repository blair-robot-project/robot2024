package frc.team449.control.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.robot2024.constants.vision.VisionConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.Optional
import kotlin.math.abs
import kotlin.math.pow

class VisionSubsystem(
  name: String,
  tagLayout: AprilTagFieldLayout,
  robotToCam: Transform3d,
  private val visionSystemSim: VisionSystemSim?
) {
  val estimator = VisionEstimator(
    tagLayout,
    PhotonCamera(name),
    robotToCam
  )

  private var lastEstTimestamp = 0.0

  private var cameraSim: PhotonCameraSim? = null

  init {
    if (RobotBase.isSimulation()) {
      visionSystemSim!!.addAprilTags(tagLayout)

      val cameraProp = SimCameraProperties()
      cameraProp.setCalibration(VisionConstants.SIM_CAMERA_WIDTH_PX, VisionConstants.SIM_CAMERA_HEIGHT_PX, Rotation2d.fromDegrees(VisionConstants.SIM_FOV_DEG))
      cameraProp.setCalibError(VisionConstants.SIM_CALIB_AVG_ERR_PX, VisionConstants.SIM_CALIB_ERR_STDDEV_PX)
      cameraProp.fps = VisionConstants.SIM_FPS
      cameraProp.avgLatencyMs = VisionConstants.SIM_AVG_LATENCY
      cameraProp.latencyStdDevMs = VisionConstants.SIM_STDDEV_LATENCY

      cameraSim = PhotonCameraSim(estimator.camera, cameraProp)

      cameraSim!!.enableDrawWireframe(VisionConstants.ENABLE_WIREFRAME)

      visionSystemSim.addCamera(cameraSim, robotToCam)
    }
  }

  private fun getSimDebugField(): Field2d? {
    return if (!RobotBase.isSimulation()) null else visionSystemSim!!.debugField
  }

  fun estimatedPose(currPose: Pose2d): Optional<EstimatedRobotPose> {
    val visionEst = estimator.estimatedPose(currPose)
    val latestTimestamp = estimator.camera.latestResult.timestampSeconds
    val newResult = abs(latestTimestamp - lastEstTimestamp) > 1e-4
    if (RobotBase.isSimulation()) {
      visionEst.ifPresentOrElse(
        { est ->
          getSimDebugField()!!
            .getObject("VisionEstimation").pose = est.estimatedPose.toPose2d()
        }
      ) { if (newResult) getSimDebugField()!!.getObject("VisionEstimation").setPoses() }
    }
    return if (newResult) {
      lastEstTimestamp = latestTimestamp
      visionEst
    } else {
      Optional.empty()
    }
  }

  fun getEstimationStdDevs(numTags: Int, avgDist: Double): Matrix<N3, N1> {
    var estStdDevs = VisionConstants.SINGLE_TAG_TRUST.copy()

    if (numTags == 0) return estStdDevs

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_TRUST.copy()

    // Increase std devs based on (average) distance
    estStdDevs.times(
      1 + avgDist.pow(VisionConstants.ORDER) * VisionConstants.PROPORTION
    )

    return estStdDevs
  }

  fun simulationPeriodic(robotSimPose: Pose2d?) {
    visionSystemSim!!.update(robotSimPose)
  }
}
