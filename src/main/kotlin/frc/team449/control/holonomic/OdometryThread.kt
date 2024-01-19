package frc.team449.control.holonomic

import edu.wpi.first.wpilibj.Notifier
import frc.team449.robot2024.constants.RobotConstants
import java.util.Queue
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier


open class OdometryThread (
  val odometryLock: ReentrantLock
  ) {
  private var signals = ArrayList<DoubleSupplier>()
  private var queues = ArrayList<Queue<Double>>()
  private var notifier = Notifier { this.periodic() }

  init {
    notifier.setName("OdometryThread")
    notifier.startPeriodic(1.0 / RobotConstants.ODOMETRY_FREQUENCY)
  }

  open fun registerSignal(signal: DoubleSupplier): Queue<Double> {
    val queue: Queue<Double> = ArrayBlockingQueue(100)
    odometryLock.lock()
    try {
      signals.add(signal)
      queues.add(queue)
    } finally {
      odometryLock.unlock()
    }
    return queue
  }

  private fun periodic() {
    odometryLock.lock()
    try {
      for (i in signals.indices) {
        queues[i].add(signals[i].asDouble)
      }
    } finally {
      odometryLock.unlock()
    }
  }
}