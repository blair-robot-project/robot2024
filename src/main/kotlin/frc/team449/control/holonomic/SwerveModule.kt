package frc.team449.control.holonomic

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import frc.team449.robot2024.constants.drives.SwerveConstants
import frc.team449.system.encoder.Encoder
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * Controls a Swerve Module.
 * @param name The name of the module (used for logging).
 * @param drivingMotor The motor that controls the speed of the module.
 * @param turningMotor The motor that controls the angle of the module
 * @param driveController The velocity control for speed of the module
 * @param turnController The position control for the angle of the module
 * @param driveFeedforward The voltage predicting equation for a given speed of the module.
 * @param location The location of the module in reference to the center of the robot.
 * NOTE: In relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModule(
  private val name: String,
  private val drivingMotor: CANSparkMax,
  private val turningMotor: CANSparkMax,
  private val turnEncoder: Encoder,
  val driveController: PIDController,
  val turnController: PIDController,
  private val driveFeedforward: SimpleMotorFeedforward,
  val location: Translation2d
) {
  init {
    turnController.enableContinuousInput(.0, 2 * PI)
    driveController.reset()
    turnController.reset()
  }

  val desiredState = SwerveModuleState(
    0.0,
    Rotation2d()
  )

  /** The module's [SwerveModuleState], containing speed and angle. */
  open var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        drivingMotor.encoder.velocity,
        Rotation2d(turnEncoder.velocity)
      )
    }
    set(desState) {
      if (abs(desState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn more than 90 degrees. */
      val optimizedState = SwerveModuleState.optimize(
        desState,
        Rotation2d(turnEncoder.position)
      )

      turnController.setpoint = optimizedState.angle.radians
      driveController.setpoint = optimizedState.speedMetersPerSecond
      desiredState.speedMetersPerSecond = optimizedState.speedMetersPerSecond
      desiredState.angle = optimizedState.angle
    }

  /** The module's [SwerveModulePosition], containing distance and angle. */
  open val position: SwerveModulePosition
    get() {
      return SwerveModulePosition(
        drivingMotor.encoder.position,
        Rotation2d(turnEncoder.position)
      )
    }

  fun setVoltage(volts: Double) {
    driveController.setpoint = 0.0
    desiredState.speedMetersPerSecond = 0.0
    turnController.setpoint = 0.0

    turningMotor.set(turnController.calculate(turnEncoder.position))
    drivingMotor.setVoltage(volts)
  }

  fun lastDrivingVoltage(): Double {
    return drivingMotor.get() * RobotController.getBatteryVoltage()
  }

  fun lastSteeringVoltage(): Double {
    return turningMotor.get() * RobotController.getBatteryVoltage()
  }

  fun appliedDutyCycleDriving(): Double {
    return drivingMotor.appliedOutput
  }

  fun appliedDutyCycleSteering(): Double {
    return turningMotor.appliedOutput
  }

  fun busVoltageDriving(): Double {
    return drivingMotor.busVoltage
  }

  fun busVoltageSteering(): Double {
    return turningMotor.busVoltage
  }

  fun outputCurrentDriving(): Double {
    return drivingMotor.outputCurrent
  }

  fun outputCurrentSteering(): Double {
    return turningMotor.outputCurrent
  }

  /** Set module speed to zero but keep module angle the same. */
  fun stop() {
    turnController.setpoint = turnEncoder.position
    desiredState.speedMetersPerSecond = 0.0
  }

  open fun update() {
    /** CONTROL speed of module */
    val drivePid = driveController.calculate(
      drivingMotor.encoder.velocity
    )
    val driveFF = driveFeedforward.calculate(
      desiredState.speedMetersPerSecond
    )
    drivingMotor.setVoltage(drivePid + driveFF)

    /** CONTROL direction of module */
    val turnPid = turnController.calculate(
      turnEncoder.position
    )

    turningMotor.set(
      turnPid +
        sign(desiredState.angle.radians - turnEncoder.position) *
        SwerveConstants.STEER_KS / RobotController.getBatteryVoltage()
    )
  }

  companion object {
    /** Create a real or simulated [SwerveModule] based on the simulation status of the robot. */
    fun create(
      name: String,
      drivingMotor: CANSparkMax,
      turningMotor: CANSparkMax,
      turnEncoder: Encoder,
      driveController: PIDController,
      turnController: PIDController,
      driveFeedforward: SimpleMotorFeedforward,
      location: Translation2d
    ): SwerveModule {
      if (RobotBase.isReal()) {
        return SwerveModule(
          name,
          drivingMotor,
          turningMotor,
          turnEncoder,
          driveController,
          turnController,
          driveFeedforward,
          location
        )
      } else {
        return SwerveModuleSim(
          name,
          drivingMotor,
          turningMotor,
          turnEncoder,
          driveController,
          turnController,
          driveFeedforward,
          location
        )
      }
    }
  }
}

/** A "simulated" swerve module. Immediately reaches to its desired state. */
class SwerveModuleSim(
  name: String,
  drivingMotor: CANSparkMax,
  turningMotor: CANSparkMax,
  turnEncoder: Encoder,
  driveController: PIDController,
  turnController: PIDController,
  driveFeedforward: SimpleMotorFeedforward,
  location: Translation2d
) : SwerveModule(
  name,
  drivingMotor,
  turningMotor,
  turnEncoder,
  driveController,
  turnController,
  driveFeedforward,
  location
) {
  // TODO: Sim needs integrating with refactored motor system
  private var turnAngle = Rotation2d(0.0)
  private var drivePosition = 0.0
  private var driveVelocity = 0.0
  private var prevTime = Timer.getFPGATimestamp()
  override var state: SwerveModuleState
    get() = SwerveModuleState(
      driveVelocity,
      turnAngle
    )
    set(desiredState) {
      super.state = desiredState
      turnAngle = desiredState.angle
      driveVelocity = desiredState.speedMetersPerSecond
    }
  override val position: SwerveModulePosition
    get() = SwerveModulePosition(
      drivePosition,
      turnAngle
    )

  override fun update() {
    val currTime = Timer.getFPGATimestamp()
    drivePosition += driveVelocity * (currTime - prevTime)
    prevTime = currTime
  }
}
