package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import frc.team449.robot2023.constants.drives.SwerveConstants
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.WrappedMotor
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
  private val drivingMotor: WrappedMotor,
  private val turningMotor: WrappedMotor,
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
        drivingMotor.velocity,
        Rotation2d(turningMotor.position)
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
        Rotation2d(turningMotor.position)
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
        drivingMotor.position,
        Rotation2d(turningMotor.position)
      )
    }

  fun setVoltage(volts: Double) {
    driveController.setpoint = 0.0
    desiredState.speedMetersPerSecond = 0.0
    turnController.setpoint = 0.0

    turningMotor.set(turnController.calculate(turningMotor.position))
    drivingMotor.setVoltage(volts)
  }

  fun lastDrivingVoltage(): Double {
    return drivingMotor.lastVoltage
  }

  fun lastSteeringVoltage(): Double {
    return turningMotor.lastVoltage
  }

  fun requestedDutyCycleDriving(): Double {
    return drivingMotor.getDutyCycle?.asDouble ?: Double.NaN
  }

  fun requestedDutyCycleSteering(): Double {
    return turningMotor.getDutyCycle?.asDouble ?: Double.NaN
  }

  fun appliedDutyCycleDriving(): Double {
    return drivingMotor.appliedOutput?.asDouble ?: Double.NaN
  }

  fun appliedDutyCycleSteering(): Double {
    return turningMotor.appliedOutput?.asDouble ?: Double.NaN
  }

  fun busVoltageDriving(): Double {
    return drivingMotor.busVoltage?.asDouble ?: Double.NaN
  }

  fun busVoltageSteering(): Double {
    return turningMotor.busVoltage?.asDouble ?: Double.NaN
  }

  fun outputCurrentDriving(): Double {
    return drivingMotor.outputCurrent?.asDouble ?: Double.NaN
  }

  fun outputCurrentSteering(): Double {
    return turningMotor.outputCurrent?.asDouble ?: Double.NaN
  }

  /** Set module speed to zero but keep module angle the same. */
  fun stop() {
    turnController.setpoint = turningMotor.position
    desiredState.speedMetersPerSecond = 0.0
  }

  open fun update() {
    /** CONTROL speed of module */
    val drivePid = driveController.calculate(
      drivingMotor.velocity
    )
    val driveFF = driveFeedforward.calculate(
      desiredState.speedMetersPerSecond
    )
    drivingMotor.setVoltage(drivePid + driveFF)

    /** CONTROL direction of module */
    val turnPid = turnController.calculate(
      turningMotor.position
    )

    turningMotor.set(
      turnPid +
        sign(desiredState.angle.radians - turningMotor.position) *
        SwerveConstants.STEER_KS / RobotController.getBatteryVoltage()
    )
  }

  companion object {
    /** Create a real or simulated [SwerveModule] based on the simulation status of the robot. */
    fun create(
      name: String,
      drivingMotor: WrappedMotor,
      turningMotor: WrappedMotor,
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
  drivingMotor: WrappedMotor,
  turningMotor: WrappedMotor,
  driveController: PIDController,
  turnController: PIDController,
  driveFeedforward: SimpleMotorFeedforward,
  location: Translation2d
) : SwerveModule(
  name,
  drivingMotor,
  turningMotor,
  driveController,
  turnController,
  driveFeedforward,
  location
) {
  private val turningMotorEncoder = Encoder.SimController(turningMotor.encoder)
  private val driveEncoder = Encoder.SimController(drivingMotor.encoder)
  private var prevTime = Timer.getFPGATimestamp()
  override var state: SwerveModuleState
    get() = SwerveModuleState(
      driveEncoder.velocity,
      Rotation2d(turningMotorEncoder.position)
    )
    set(desiredState) {
      super.state = desiredState
      turningMotorEncoder.position = desiredState.angle.radians
      driveEncoder.velocity = desiredState.speedMetersPerSecond
    }
  override val position: SwerveModulePosition
    get() = SwerveModulePosition(
      driveEncoder.position,
      Rotation2d(turningMotorEncoder.position)
    )

  override fun update() {
    val currTime = Timer.getFPGATimestamp()
    driveEncoder.position += driveEncoder.velocity * (currTime - prevTime)
    prevTime = currTime
  }
}
