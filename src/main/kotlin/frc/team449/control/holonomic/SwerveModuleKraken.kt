package frc.team449.control.holonomic

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.team449.robot2024.constants.drives.SwerveConstantsKraken
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.WrappedNEO
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * Controls a Swerve Module.
 * @param name The name of the module (used for logging).
 * @param drivingMotor The motor that controls the speed of the module.
 * @param turningMotor The motor that controls the angle of the module
 * @param turnController The position control for the angle of the module
 * @param location The location of the module in reference to the center of the robot.
 * NOTE: In relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModuleKraken(
  private val name: String,
  private val drivingMotor: TalonFX,
  private val turningMotor: WrappedNEO,
  val turnController: PIDController,
  val location: Translation2d
) {
  init {
    turnController.enableContinuousInput(.0, 2 * PI)
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
        drivingMotor.velocity.value,
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
      desiredState.speedMetersPerSecond = optimizedState.speedMetersPerSecond
      desiredState.angle = optimizedState.angle
    }

  /** The module's [SwerveModulePosition], containing distance and angle. */
  open val position: SwerveModulePosition
    get() {
      return SwerveModulePosition(
        drivingMotor.position.value,
        Rotation2d(turningMotor.position)
      )
    }

  fun setVoltage(volts: Double) {
    desiredState.speedMetersPerSecond = 0.0
    turnController.setpoint = 0.0

    turningMotor.set(turnController.calculate(turningMotor.position))
    drivingMotor.setVoltage(volts)
  }

  /** Set module speed to zero but keep module angle the same. */
  fun stop() {
    turnController.setpoint = turningMotor.position
    desiredState.speedMetersPerSecond = 0.0
  }

  open fun update() {
    /** CONTROL speed of module */
    drivingMotor.setControl(
      VelocityVoltage(desiredState.speedMetersPerSecond / SwerveConstantsKraken.DRIVE_UPR)
        .withUpdateFreqHz(1000.0)
        .withEnableFOC(SwerveConstantsKraken.USE_FOC)
    )

    /** CONTROL direction of module */
    val turnPid = turnController.calculate(
      turningMotor.position
    )

    turningMotor.set(
      turnPid +
        sign(desiredState.angle.radians - turningMotor.position) *
          SwerveConstantsKraken.STEER_KS
    )
  }

  companion object {
    /** Create a real or simulated [SwerveModule] based on the simulation status of the robot. */
    fun create(
      name: String,
      drivingMotor: TalonFX,
      turningMotor: WrappedNEO,
      turnController: PIDController,
      location: Translation2d
    ): SwerveModuleKraken {
      if (RobotBase.isReal()) {
        return SwerveModuleKraken(
          name,
          drivingMotor,
          turningMotor,
          turnController,
          location
        )
      } else {
        return SwerveModuleSimKraken(
          name,
          drivingMotor,
          turningMotor,
          turnController,
          location
        )
      }
    }
  }
}

/** A "simulated" swerve module. Immediately reaches to its desired state. */
class SwerveModuleSimKraken(
  name: String,
  drivingMotor: TalonFX,
  turningMotor: WrappedNEO,
  turnController: PIDController,
  location: Translation2d
) : SwerveModuleKraken(
  name,
  drivingMotor,
  turningMotor,
  turnController,
  location
) {
  private val turningMotorEncoder = Encoder.SimController(turningMotor.encoder)

  private var drivePosition = 0.0
  private var driveVelocity = 0.0

  private var prevTime = Timer.getFPGATimestamp()
  override var state: SwerveModuleState
    get() = SwerveModuleState(
      driveVelocity,
      Rotation2d(turningMotorEncoder.position)
    )
    set(desiredState) {
      super.state = desiredState
      turningMotorEncoder.position = desiredState.angle.radians
      driveVelocity = desiredState.speedMetersPerSecond
    }

  override fun update() {
    val currTime = Timer.getFPGATimestamp()
    drivePosition += driveVelocity * (currTime - prevTime)
    prevTime = currTime
  }
}
