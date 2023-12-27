package frc.team449.control.differential

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds
import frc.team449.control.OI
import frc.team449.robot2023.constants.RobotConstants

/**
 * Helper class to create OIs for a differential drivetrain (arcade, curvature,
 * or tank)
 */
object DifferentialOIs {
  /**
   * Create an OI for arcade drive. One throttle controls forward-backward speed,
   * the other; controls rotation.
   *
   * @param drive The drivetrain
   * @param xThrottle Throttle to get forward-backward movement
   * @param rotThrottle Throttle to get rotation
   * @param xRamp Used for limiting forward-backward acceleration
   * @param rotRamp Used for limiting rotational acceleration
   */
  fun createArcade(
    drive: DifferentialDrive,
    xThrottle: () -> Double,
    rotThrottle: () -> Double,
    xRamp: SlewRateLimiter,
    rotRamp: SlewRateLimiter
  ): OI = OI {
    scaleAndApplyRamping(
      edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK(
        xThrottle(),
        rotThrottle(),
        false
      ),
      drive.kinematics,
      xRamp,
      rotRamp
    )
  }

  /**
   * Create OI for curvature drive (drives like a car). One throttle controls
   * forward-backward speed, like arcade, but the other controls curvature instead
   * of rotation. Ramping is still applied to rotation instead of curvature.
   *
   * @param drive The drivetrain
   * @param xThrottle Throttle to get forward-backward movement
   * @param rotThrottle Throttle to get rotation
   * @param xRamp Used for limiting forward-backward acceleration
   * @param rotRamp Used for limiting rotational acceleration
   * @param turnInPlace When this returns true, turns in place instead of turning
   *        like a car
   */
  fun createCurvature(
    drive: DifferentialDrive,
    xThrottle: () -> Double,
    rotThrottle: () -> Double,
    xRamp: SlewRateLimiter,
    rotRamp: SlewRateLimiter,
    turnInPlace: () -> Boolean
  ): OI = OI {
    scaleAndApplyRamping(
      edu.wpi.first.wpilibj.drive.DifferentialDrive.curvatureDriveIK(
        xThrottle(),
        rotThrottle(),
        turnInPlace()
      ),
      drive.kinematics,
      xRamp,
      rotRamp
    )
  }

  /**
   * Create an OI for tank drive. Each throttle controls one side of the drive
   * separately. Each side is also ramped separately.
   *
   * <p>
   * Shame on you if you ever use this.
   *
   * @param drive The drivetrain
   * @param leftThrottle Throttle to get forward-backward movement
   * @param rightThrottle Throttle to get rotation
   * @param leftRamp Used for limiting the left side's acceleration
   * @param rightRamp Used for limiting the right side's acceleration
   */
  fun createTank(
    drive: DifferentialDrive,
    leftThrottle: () -> Double,
    rightThrottle: () -> Double,
    leftRamp: SlewRateLimiter,
    rightRamp: SlewRateLimiter
  ): OI = OI {
    drive.kinematics.toChassisSpeeds(
      DifferentialDriveWheelSpeeds(
        leftRamp.calculate(leftThrottle() * RobotConstants.MAX_LINEAR_SPEED),
        rightRamp.calculate(
          rightThrottle() * RobotConstants.MAX_LINEAR_SPEED
        )
      )
    )
  }

  /**
   * Scales differential drive throttles from [-1, 1] to [-maxSpeed, maxSpeed], then
   * applies ramping to give the final [ChassisSpeeds].
   *
   * <p>
   * Do note that although this is given a [DifferentialDriveWheelSpeeds]
   * object, the ramping isn't applied to the left and right side but to the
   * linear and rotational velocity using a [ChassisSpeeds] object.
   *
   * @param wheelThrottles The left and right wheel throttles
   * @param kinematics Kinematics object used for turning differential drive wheel
   *        speeds to chassis speeds
   * @param ramp Used for limiting linear/forward-back acceleration
   * @param rotRamp Used for limiting rotational acceleration
   */
  private fun scaleAndApplyRamping(
    wheelThrottles: WheelSpeeds,
    kinematics: DifferentialDriveKinematics,
    ramp: SlewRateLimiter,
    rotRamp: SlewRateLimiter
  ): ChassisSpeeds {
    val leftVel = wheelThrottles.left * RobotConstants.MAX_LINEAR_SPEED
    val rightVel = wheelThrottles.right * RobotConstants.MAX_LINEAR_SPEED
    val chassisSpeeds = kinematics.toChassisSpeeds(
      DifferentialDriveWheelSpeeds(leftVel, rightVel)
    )
    return ChassisSpeeds(
      ramp.calculate(chassisSpeeds.vxMetersPerSecond),
      0.0,
      rotRamp.calculate(chassisSpeeds.omegaRadiansPerSecond)
    )
  }
}
