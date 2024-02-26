package frc.team449.robot2024.constants.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import frc.team449.control.vision.VisionSubsystem
import org.photonvision.PhotonCamera
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.VisionSystemSim

/** Constants that have anything to do with vision */
object VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TEST_TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(3, Pose3d())
    ),
    16.4846,
    8.1026
  )

  /** WPILib's AprilTagFieldLayout for the 2023 Charged Up Game */
  val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
    AprilTagFields.k2024Crescendo.m_resourceFile
  )

  /** Object Detection */
  val noteCam = PhotonCamera("note-cam")

  const val CAMERA_HEIGHT_METERS = 0.0
  const val TARGET_HEIGHT_METERS = 0.0
  const val CAMERA_PITCH_RADIANS = 0.0

  /** Robot to Camera Distance */
  val frontLeft = Transform3d(
    Translation3d(Units.inchesToMeters(9.143), Units.inchesToMeters(12.662), Units.inchesToMeters(15.4953)),
    Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(45.0))
  )
  val frontRight = Transform3d(
    Translation3d(Units.inchesToMeters(9.143), Units.inchesToMeters(-12.662), Units.inchesToMeters(15.4953)),
    Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(-45.0))
  )
  val backLeft = Transform3d(
    Translation3d(Units.inchesToMeters(-10.696), Units.inchesToMeters(10.848), Units.inchesToMeters(9.11)),
    Rotation3d(0.0, Units.degreesToRadians(-28.125), Units.degreesToRadians(180.0 + 7.5))
  )
  val backRight = Transform3d(
    Translation3d(Units.inchesToMeters(-10.696), Units.inchesToMeters(-10.848), Units.inchesToMeters(9.11)),
    Rotation3d(0.0, Units.degreesToRadians(-28.125), Units.degreesToRadians(180.0 - 7.5))
  )

  val TAG_MODEL = TargetModel(
    Units.inchesToMeters(6.5),
    Units.inchesToMeters(6.5)
  )

  /** Filtering Constants */
  const val MAX_AMBIGUITY = 0.325
  const val MAX_DISTANCE_SINGLE_TAG = 3.0
  const val MAX_DISTANCE_MULTI_TAG = 5.0
  const val SINGLE_TAG_HEADING_MAX_DEV_DEG = 5.0
  var MAX_HEIGHT_ERR_METERS = 0.75
  const val NUM_TAG_FACTOR = 2.0

  /** Std Dev Calculation Constants */
  const val ORDER = 2
  const val PROPORTION = 1.0 / 20.0

  val VISION_SIM = VisionSystemSim(
    "main"
  )

  /** Vision Sim Setup Constants */
  const val SIM_FPS = 13.0
  const val SIM_CAMERA_HEIGHT_PX = 720
  const val SIM_CAMERA_WIDTH_PX = 1280
  const val SIM_FOV_DEG = 75.0
  const val SIM_CALIB_AVG_ERR_PX = 0.35
  const val SIM_CALIB_ERR_STDDEV_PX = 0.10
  const val SIM_AVG_LATENCY = 50.0
  const val SIM_STDDEV_LATENCY = 10.0
  const val ENABLE_WIREFRAME = true

  /** List of cameras that we want to use */
  val ESTIMATORS: ArrayList<VisionSubsystem> = arrayListOf(
//    VisionSubsystem(
//      "back_left",
//      TAG_LAYOUT,
//      backLeft,
//      VISION_SIM
//    ),
//    VisionSubsystem(
//      "back_right",
//      TAG_LAYOUT,
//      backRight,
//      VISION_SIM
//    ),
//    VisionSubsystem(
//      "front_left",
//      TAG_LAYOUT,
//      frontLeft,
//      VISION_SIM
//    ),
//    VisionSubsystem(
//      "front_right",
//      TAG_LAYOUT,
//      frontRight,
//      VISION_SIM
//    ),
  )

  val ENCODER_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .20, .20, .015)
  val SINGLE_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .225, .225, 1e+9)
  val MULTI_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .175, .175, .80)
}
