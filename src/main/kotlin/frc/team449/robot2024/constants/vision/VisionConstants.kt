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

  /** Robot to Camera distance */
  val robotToCamera1 = Transform3d(
    Translation3d(Units.inchesToMeters(12.696906), Units.inchesToMeters(11.957), Units.inchesToMeters(11.635)),
    Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(180.0))
  )
  val robotToCamera2 = Transform3d(
    Translation3d(Units.inchesToMeters(10.579978), Units.inchesToMeters(-7.928648), Units.inchesToMeters(5.336082)),
    Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(180.0))
  )
  val robotToCamera3 = Transform3d(
    Translation3d(Units.inchesToMeters(-10.580250), Units.inchesToMeters(-7.928502), Units.inchesToMeters(5.336028)),
    Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(180.0))
  )
  val robotToCamera4 = Transform3d(
    Translation3d(Units.inchesToMeters(-12.696906), Units.inchesToMeters(11.957), Units.inchesToMeters(11.635)),
    Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(180.0))
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
  const val NUM_TAG_FACTOR = 2.0

  /** Std Dev Calculation Constants */
  const val ORDER = 1
  const val PROPORTION = 1 / 2

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
      "spinel 1",
      TAG_LAYOUT,
      robotToCamera1,
      VISION_SIM
    ),
    VisionSubsystem(
      "spinel 2",
      TAG_LAYOUT,
      robotToCamera2,
      VISION_SIM
    ),
    VisionSubsystem(
      "spinel 3",
      TAG_LAYOUT,
      robotToCamera3,
      VISION_SIM
    ),
    VisionSubsystem(
      "spinel 4",
      TAG_LAYOUT,
      robotToCamera4,
      VISION_SIM
    ),
//    VisionSubsystem(
//      "arducam",
//      TAG_LAYOUT,
//      robotToCamera,
//      VISION_SIM
//    ),
//    VisionSubsystem(
//      "arducam2",
//      TAG_LAYOUT,
//      Transform3d(
//        Translation3d(Units.inchesToMeters(11.48657), Units.inchesToMeters(-10.0), Units.inchesToMeters(8.3416)),
//        Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(-45.0))
//      ),
//      VISION_SIM
//    ),
//    VisionSubsystem(
//      "arducam3",
//      TAG_LAYOUT,
//      Transform3d(
//        Translation3d(Units.inchesToMeters(11.48657), Units.inchesToMeters(10.0), Units.inchesToMeters(8.3416)),
//        Rotation3d(0.0, Units.degreesToRadians(-15.0), Units.degreesToRadians(45.0))
//      ),
//      VISION_SIM
//    )
  )

  val ENCODER_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .125, .125, .015)
  val SINGLE_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .125, .125, .80)
  val MULTI_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .055, .055, .30)
}
