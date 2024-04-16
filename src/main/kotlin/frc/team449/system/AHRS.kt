package frc.team449.system

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import frc.team449.util.simBooleanProp
import frc.team449.util.simDoubleProp

class AHRS(
  val navx: com.kauailabs.navx.frc.AHRS
) {

  var prevPos = Double.NaN
  var prevTime = Double.NaN

  private val filter = LinearFilter.movingAverage(5)

  /** The current reading of the gyro with the offset included */
  val heading: Rotation2d
    get() {
      return -Rotation2d.fromDegrees(navx.fusedHeading.toDouble())
    }

  val pitch: Rotation2d
    get() {
      return -Rotation2d.fromDegrees(navx.pitch.toDouble())
    }

  val roll: Rotation2d
    get() {
      return -Rotation2d.fromDegrees(navx.roll.toDouble())
    }

  fun angularXVel(): Double {
    val currPos = navx.roll.toDouble()
    val currTime = Timer.getFPGATimestamp()

    val vel = if (prevPos.isNaN()) {
      0.0
    } else {
      val dt = currTime - prevTime
      val dx = currPos - prevPos
      dx / dt
    }
    this.prevTime = currTime
    this.prevPos = currPos
    return filter.calculate(vel)
  }

  constructor(
    port: SPI.Port = SPI.Port.kMXP
  ) : this(
    com.kauailabs.navx.frc.AHRS(port)
  )

  fun calibrated(): Boolean {
    return !navx.isCalibrating
  }

  fun connected(): Boolean {
    return navx.isConnected
  }

  /**
   * Used to set properties of an [AHRS] object during simulation. See
   * https://pdocs.kauailabs.com/navx-mxp/softwa
   * re/roborio-libraries/java/
   *
   * @param devName The name of the simulated device.
   * @param index The NavX index.
   */
  class SimController(devName: String = "navX-Sensor", index: Int = 0) {
    private val deviceSim = SimDeviceSim(devName, index)

    var isConnected by simBooleanProp(deviceSim.getBoolean("Connected"))
    var yaw by simDoubleProp(deviceSim.getDouble("Yaw"))
    var pitch by simDoubleProp(deviceSim.getDouble("Pitch"))
    var roll by simDoubleProp(deviceSim.getDouble("Roll"))
    var compassHeading by simDoubleProp(deviceSim.getDouble("CompassHeading"))
    var fusedHeading by simDoubleProp(deviceSim.getDouble("FusedHeading"))
    var linearWorldAccelX by simDoubleProp(deviceSim.getDouble("LinearWorldAccelX"))
    var linearWorldAccelY by simDoubleProp(deviceSim.getDouble("LinearWorldAccelX"))
    var linearWorldAccelZ by simDoubleProp(deviceSim.getDouble("LinearWorldAccelY"))
  }
}
