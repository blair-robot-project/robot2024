package frc.team449.robot2023.constants.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
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
import edu.wpi.first.wpilibj.Filesystem
import frc.team449.control.vision.VisionSubsystem
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
  val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(
    Filesystem.getDeployDirectory().absolutePath.plus("/vision/Bunnybots2023.json")
  )

//    AprilTagFieldLayout.loadFromResource(
//    AprilTagFields.k2023ChargedUp.m_resourceFile
//  )

  /** Robot to Camera distance */
  val robotToCamera = Transform3d(
    Translation3d(Units.inchesToMeters(-11.48657), Units.inchesToMeters(0.0), Units.inchesToMeters(8.3416)),
    Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(180.0))
  )

  val TAG_MODEL = TargetModel(
    Units.inchesToMeters(6.5),
    Units.inchesToMeters(6.5)
  )

  /** Filtering Constants */
  const val MAX_AMBIGUITY = 0.325
  const val MAX_DISTANCE_SINGLE_TAG = 3.45
  const val MAX_DISTANCE_MULTI_TAG = 4.5
  const val SINGLE_TAG_HEADING_MAX_DEV_DEG = 5.0
  const val MAX_HEIGHT_ERR_METERS = 0.075
  const val TAG_MULTIPLIER = 0.5

  /** Std Dev Calculation Constants */
  const val ORDER = 2
  const val PROPORTION = 1 / 25

  val VISION_SIM = VisionSystemSim(
    "main"
  )

  /** Vision Sim Setup Constants */
  const val SIM_FPS = 25.0
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
    VisionSubsystem(
      "arducam",
      TAG_LAYOUT,
      robotToCamera,
      VISION_SIM
    )
  )

  val ENCODER_TRUST: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(.085, .085, .015)
  val SINGLE_TAG_TRUST: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(.125, .125, .80)
  val MULTI_TAG_TRUST: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(.0275, .0275, .30)

}
