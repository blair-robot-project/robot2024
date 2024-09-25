package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.routines.RoutineChooser
import frc.team449.robot2024.commands.PivotCalibration
import frc.team449.robot2024.commands.light.BlairChasing
import frc.team449.robot2024.commands.light.BreatheHue
import frc.team449.robot2024.commands.light.Rainbow
import frc.team449.robot2024.constants.field.FieldConstants
import frc.team449.robot2024.constants.vision.VisionConstants
import frc.team449.robot2024.subsystems.NewControllerBindings
import monologue.Annotations.Log
import monologue.Logged
import monologue.Monologue
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot(), Logged {

  @Log.NT
  private val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  @Log.NT
  private val field = robot.field
  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = NewControllerBindings(robot.driveController, robot.mechController, robot)

  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    if (RobotBase.isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
//      val instance = NetworkTableInstance.getDefault()
//      instance.stopServer()
//      instance.startClient4("localhost")
    }

    PivotCalibration(robot.pivot).ignoringDisable(true).schedule()

    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    routineMap = routineChooser.routineMap()
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    routineChooser.createOptions()

    SmartDashboard.putData("Routine Chooser", routineChooser)
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    robot.light.defaultCommand = BlairChasing(robot.light)

    controllerBinder.bindButtons()

    DriverStation.startDataLog(DataLogManager.getLog())
    Monologue.setupMonologue(this, "/Monologuing", false, false)

    URCL.start()
  }

  override fun driverStationConnected() {
    Monologue.setFileOnly(DriverStation.isFMSAttached())
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    robot.field.robotPose = robot.drive.pose

    robot.field.getObject("bumpers").pose = robot.drive.pose

    Monologue.updateAll()
  }

  override fun autonomousInit() {
    /** Every time auto starts, we update the chosen auto command. */
    this.autoCommand = routineMap[if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) "Red" + routineChooser.selected else "Blue" + routineChooser.selected]
    CommandScheduler.getInstance().schedule(this.autoCommand)

    robot.drive.enableVisionFusion = false

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      BreatheHue(robot.light, 0).schedule()
    } else {
      BreatheHue(robot.light, 95).schedule()
    }

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      FieldConstants.SPEAKER_POSE = FieldConstants.RED_SPEAKER_POSE
    } else if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
      FieldConstants.SPEAKER_POSE = FieldConstants.BLUE_SPEAKER_POSE
    }

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      FieldConstants.PASS_POSE = FieldConstants.RED_PASS_POSE
    } else if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
      FieldConstants.PASS_POSE = FieldConstants.BLUE_PASS_POSE
    }
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }

    robot.drive.enableVisionFusion = true

    (robot.light.currentCommand ?: InstantCommand()).cancel()

    robot.drive.defaultCommand = robot.driveCommand

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      FieldConstants.SPEAKER_POSE = FieldConstants.RED_SPEAKER_POSE
    } else if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
      FieldConstants.SPEAKER_POSE = FieldConstants.BLUE_SPEAKER_POSE
    }

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      FieldConstants.PASS_POSE = FieldConstants.RED_PASS_POSE
    } else if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
      FieldConstants.PASS_POSE = FieldConstants.BLUE_PASS_POSE
    }
  }

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {
    robot.drive.stop()

    robot.drive.enableVisionFusion = true

    (robot.light.currentCommand ?: InstantCommand()).cancel()
    Rainbow(robot.light).schedule()
  }

  override fun disabledPeriodic() {}

  override fun testInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }
  }

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {
    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.drive.odoPose)
    }

    VisionConstants.VISION_SIM.debugField.getObject("EstimatedRobot").pose = robot.drive.pose
  }
}
