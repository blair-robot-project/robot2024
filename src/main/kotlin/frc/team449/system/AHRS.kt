package frc.team449.system

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import frc.team449.control.holonomic.OdometryThread
import frc.team449.util.simBooleanProp
import frc.team449.util.simDoubleProp
import java.util.concurrent.locks.ReentrantLock

class AHRS(
  private val navx: com.kauailabs.navx.frc.AHRS,
  private val odometryLock: ReentrantLock
) {

  var prevPos = Double.NaN
  var prevTime = Double.NaN
  val odometryThread = OdometryThread(odometryLock)
  val pitchQueue = odometryThread.registerSignal { navx.pitch.toDouble() }
  val rollQueue = odometryThread.registerSignal { navx.roll.toDouble() }
  val headingQueue = odometryThread.registerSignal { navx.fusedHeading.toDouble() }

  private val filter = LinearFilter.movingAverage(5)


  /** The current reading of the gyro with the offset included */
  val heading: Rotation2d
    get() {
      odometryLock.lock()
      val output = -Rotation2d.fromDegrees(headingQueue.peek())
      odometryLock.unlock()
      return output
    }

  val pitch: Rotation2d
    get() {
      odometryLock.lock()
      val output = -Rotation2d.fromDegrees(pitchQueue.peek())
      odometryLock.unlock()
      return output
    }

  val roll: Rotation2d
    get() {
      odometryLock.lock()
      val output = -Rotation2d.fromDegrees(rollQueue.peek())
      odometryLock.unlock()
      return output
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
    port: SPI.Port = SPI.Port.kMXP,
    lock: ReentrantLock
  ) : this(
    com.kauailabs.navx.frc.AHRS(port),
    lock
  )

  fun calibrated(): Boolean {
    return navx.isMagnetometerCalibrated
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
