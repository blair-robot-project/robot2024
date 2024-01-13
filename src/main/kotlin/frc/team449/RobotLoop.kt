package frc.team449

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.holonomic.SwerveSim
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.routines.RoutineChooser
import frc.team449.robot2024.commands.light.BlairChasing
import frc.team449.robot2024.commands.light.BreatheHue
import frc.team449.robot2024.commands.light.Rainbow
import frc.team449.robot2024.constants.vision.VisionConstants
import frc.team449.robot2024.subsystems.ControllerBindings
import monologue.Annotations
import monologue.Logged
import monologue.Monologue
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
class RobotLoop : TimedRobot(), Logged {

  @Annotations.Log.NT
  private val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  @Annotations.Log.NT
  private val field = robot.field

  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)

  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    if (RobotBase.isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
//      val instance = NetworkTableInstance.getDefault()
//      instance.stopServer()
//      instance.startClient4("localhost")
    }

    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    routineMap = routineChooser.routineMap()
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    SmartDashboard.putData("Routine Chooser", routineChooser)
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())
    SmartDashboard.putBoolean("Enable Logging?", false)

    robot.light.defaultCommand = BlairChasing(robot.light)

    controllerBinder.bindButtons()

    Monologue.setupMonologue(this, "/Monologuing", false, false)
//    URCL.start()
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    robot.field.robotPose = robot.drive.pose

    robot.field.getObject("bumpers").pose = robot.drive.pose

    Monologue.setFileOnly(DriverStation.isFMSAttached())
    Monologue.updateAll()
  }

  override fun autonomousInit() {
    VisionConstants.ENCODER_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .0225, .0225, .010))

    /** Every time auto starts, we update the chosen auto command */
    this.autoCommand = routineMap[routineChooser.selected]
    CommandScheduler.getInstance().schedule(this.autoCommand)

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      BreatheHue(robot.light, 0).schedule()
    } else {
      BreatheHue(robot.light, 95).schedule()
    }
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    VisionConstants.ENCODER_TRUST.setColumn(0, MatBuilder.fill(Nat.N3(), Nat.N1(), .085, .085, .015))

    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }

    (robot.light.currentCommand ?: InstantCommand()).cancel()

    robot.drive.defaultCommand = robot.driveCommand
  }

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {
    robot.drive.stop()

    (robot.light.currentCommand ?: InstantCommand()).cancel()
    Rainbow(robot.light).schedule()
  }

  override fun disabledPeriodic() {
    routineChooser.updateOptions(DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red)
  }

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
