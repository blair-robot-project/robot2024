package frc.team449.control

import edu.wpi.first.math.util.Units
import frc.team449.robot2023.constants.MotorConstants
import kotlin.math.*

// TODO: Copy parameter descriptions from Rafi's spreadsheet
//  and figure out to do for current vel > 0 situations (ideas: don't do anything, or use a trap profile in that case idk)

/** @param effectiveGearing is motor rotations : output rotations (should be greater than 1 if a reduction
 * @param pulleyRadius in meters
 * @param systemMass in kilograms
 * @param angle in degrees
 * @param tolerance in meters
 * @param vMax user specified max speed in m/s
 * @param startingDistance the start distance of the profile
 * @param finalDistance the goal distance of the profile
 * @param aStop the max negative acceleration of the profile
 * @param efficiency percent efficiency of the system
 */
class TrapezoidalExponentialProfile(
  pulleyRadius: Double,
  currentLimit: Int,
  numMotors: Int,
  effectiveGearing: Double,
  systemMass: Double,
  angle: Double,
  private val tolerance: Double = 0.05,
  private val vMax: Double = 5.0,
  startingDistance: Double = 0.0,
  private var finalDistance: Double,
  private var aStop: Double = 9.81,
  efficiency: Double
) {
  private val trueStartingDistance: Double = startingDistance
  private val trueFinalDistance: Double = finalDistance
  private var switchedStartingAndFinal: Boolean = false

  // NEO Motor Constants
  val freeSpeed = MotorConstants.FREE_SPEED
  val freeCurrent = MotorConstants.FREE_CURRENT
  val stallCurrent = MotorConstants.STALL_CURRENT
  val stallTorque = MotorConstants.STALL_TORQUE * efficiency

  private fun expDecelIntercept(
    vFree: Double,
    vLim: Double,
    aLim: Double,
    aStop: Double,
    xF: Double,
    x20: Double
  ): Double {
    val A = vFree
    val B = -(vFree - vLim)
    val C = -aLim / (vFree - vLim)
    val D = 2 * aStop
    val deltaX = xF - x20

    var lastSeed = -5.0
    var seed = 0.0
    var seedVal = evaluateExp(A, B, C, D, deltaX, seed)
    var seedDer = evaluateExpDerivative(A, B, C, D, seed)

    while (abs(seed - lastSeed) > 1e-4) {
      lastSeed = seed
      seed -= seedVal / seedDer
      seedVal = evaluateExp(A, B, C, D, deltaX, seed)
      seedDer = evaluateExpDerivative(A, B, C, D, seed)

      if (seedVal == 0.0) {
        return 0.0
      }

      if (seedDer == 0.0) {
        error("Newton method derivative is 0")
      }
    }

    return seed
  }

  private fun evaluateExp(
    A: Double,
    B: Double,
    C: Double,
    D: Double,
    deltaX: Double,
    t: Double
  ): Double {
    return B.pow(2) * exp(2 * C * t) + (D * B / C + 2 * A * B) * exp(C * t) + A * D * t - D * B / C - deltaX * D + A.pow(
      2
    )
  }

  private fun evaluateExpDerivative(
    A: Double,
    B: Double,
    C: Double,
    D: Double,
    t: Double
  ): Double {
    return 2 * B.pow(2) * C * exp(2 * C * t) + (B * D / C + 2 * A * B) * C * exp(C * t) + A * D
  }

  private val vLim =
    pulleyRadius * freeSpeed / effectiveGearing * (1 - (currentLimit - freeCurrent) / (stallCurrent - freeCurrent))
  private val effectiveStallTorque = stallTorque * numMotors
  private val effectiveGravity = 9.81 * sin(Units.degreesToRadians(angle))
  private var aLim =
    ((currentLimit - freeCurrent) / (stallCurrent - freeCurrent) * effectiveStallTorque * effectiveGearing) / (systemMass * pulleyRadius) - effectiveGravity
  private val vFree =
    pulleyRadius * freeSpeed / effectiveGearing * (1 - systemMass * effectiveGravity * pulleyRadius / effectiveGearing / effectiveStallTorque)
  private val vMax2 = min(vMax, vFree)

  init {
    if (startingDistance > finalDistance) {
      finalDistance = trueStartingDistance - trueFinalDistance
      val buffer = aStop
      aStop = aLim
      aLim = buffer
      switchedStartingAndFinal = true
    } else if (startingDistance > 0) {
      finalDistance = trueFinalDistance - trueStartingDistance
    }
  }

  private val t12 = vLim / aLim
  private val t13 = vMax2 / aLim
  private val t14 = sqrt(2 * finalDistance / (aLim + aLim.pow(2) / aStop))
  private val t1f = min(t14, min(t12, t13))
  private val x1f = 0.5 * aLim * t1f.pow(2)
  private val v1f = aLim * t1f

  private val ENTER_EXP = t12 < t13 && t12 < t14
  private val t20 = if (ENTER_EXP) t12 else Double.NaN
  private val x20 = if (ENTER_EXP) x1f else Double.NaN
  private val v20 = if (ENTER_EXP) v1f else Double.NaN
  private val dt23 =
    if ((vFree - vMax2) / (vFree - vLim) > 0) -(vFree - vLim) / aLim * ln((vFree - vMax2) / (vFree - vLim)) else Double.NaN
  private val dt24 = if (ENTER_EXP) expDecelIntercept(vFree, vLim, aLim, aStop, finalDistance, x20) else Double.NaN
  private val t2f = t20 + if (!dt23.isNaN() && !dt24.isNaN()) min(dt23, dt24) else if (!dt24.isNaN()) dt24 else 0.0
  private val x2f =
    x20 + vFree * (t2f - t20) + (vFree - vLim).pow(2) / aLim * (exp(-aLim / (vFree - vLim) * (t2f - t20)) - 1)
  private val v2f = vFree - (vFree - vLim) * exp(-aLim / (vFree - vLim) * (t2f - t20))
  private val ENTER_COAST =
    if (ENTER_EXP) {
      if (!dt23.isNaN() && !dt24.isNaN()) {
        dt24 > dt23
      } else {
        false
      }
    } else {
      t14 > t13
    }

  private val t30 =
    if (ENTER_COAST) {
      if (ENTER_EXP) {
        t2f
      } else {
        t13
      }
    } else {
      Double.NaN
    }

  private val x30 =
    if (ENTER_COAST) {
      if (ENTER_EXP) {
        x2f
      } else {
        x1f
      }
    } else {
      Double.NaN
    }

  private val v30 =
    if (ENTER_COAST) {
      if (ENTER_EXP) {
        v2f
      } else {
        v1f
      }
    } else {
      Double.NaN
    }

  private val dt34 = if (!x30.isNaN()) (finalDistance - x30) - vMax2 - vMax2 / 2 / aStop else Double.NaN
  private val t3f = if (!dt34.isNaN() && !t30.isNaN()) dt34 + t30 else Double.NaN
  private val x3f = if (!x30.isNaN() && !dt34.isNaN()) x30 + vMax2 * dt34 else Double.NaN
  private val v3f = vMax2

  private val t40 = if (ENTER_COAST) t3f else if (ENTER_EXP) t2f else t14
  private val x40 = if (ENTER_COAST) x3f else if (ENTER_EXP) x2f else x1f
  private val v40 = if (ENTER_COAST) v3f else if (ENTER_EXP) v2f else v1f
  private val t4f = t40 + v40 / aStop
  private val x4f = x40 + v40 * (t4f - t40) - 0.5 * aStop * (t4f - t40).pow(2)
  private val v4f = v40 - aStop * (t4f - t40)

  val finalTime = t4f

  private fun sample1(t: Double): Pair<Double, Double> {
    return if (switchedStartingAndFinal) {
      Pair(
        trueStartingDistance - (0.5 * aLim * t.pow(2)),
        -(aLim * t)
      )
    } else {
      Pair(
        0.5 * aLim * t.pow(2) + trueStartingDistance,
        aLim * t
      )
    }
  }

  private fun sample2(t: Double): Pair<Double, Double> {
    return if (switchedStartingAndFinal) {
      Pair(
        trueStartingDistance - (x20 + vFree * (t - t20) + (vFree - vLim).pow(2) / aLim * (exp(-aLim / (vFree - vLim) * (t - t20)) - 1)),
        -(vFree - (vFree - vLim) * exp(-aLim / (vFree - vLim) * (t - t20)))
      )
    } else {
      Pair(
        x20 + vFree * (t - t20) + (vFree - vLim).pow(2) / aLim * (exp(-aLim / (vFree - vLim) * (t - t20)) - 1) + trueStartingDistance,
        vFree - (vFree - vLim) * exp(-aLim / (vFree - vLim) * (t - t20))
      )
    }
  }

  private fun sample3(t: Double): Pair<Double, Double> {
    return if (switchedStartingAndFinal) {
      Pair(
        // hey, this was vMax in this script. Shouldn't this be vMax2 for the actual top speed?
        trueStartingDistance - (x30 + vMax2 * (t - t30)),
        -vMax
      )
    } else {
      Pair(
        // hey, this was vMax in this script. Shouldn't this be vMax2 for the actual top speed?
        x30 + vMax2 * (t - t30) + trueStartingDistance,
        vMax
      )
    }
  }

  private fun sample4(t: Double): Pair<Double, Double> {
    return if (switchedStartingAndFinal) {
      Pair(
        trueStartingDistance - (x40 + v40 * (t - t40) - 0.5 * aStop * (t - t40).pow(2)),
        -(v40 - aStop * (t - t40))
      )
    } else {
      Pair(
        x40 + v40 * (t - t40) - 0.5 * aStop * (t - t40).pow(2) + trueStartingDistance,
        v40 - aStop * (t - t40)
      )
    }
  }

  fun calculate(t: Double): Pair<Double, Double> {
    if (abs(trueFinalDistance - trueStartingDistance) < tolerance) return Pair(trueFinalDistance, 0.0)

    if (t <= 0.0) return Pair(trueStartingDistance, 0.0)

    if (t >= t4f) return Pair(trueFinalDistance, 0.0)

    if (t <= t1f) {
      return sample1(t)
    }

    if (ENTER_EXP) {
      return if (ENTER_COAST) {
        if (t <= t2f) {
          sample2(t)
        } else if (t <= t3f) {
          sample3(t)
        } else {
          sample4(t)
        }
      } else {
        if (t <= t2f) {
          sample2(t)
        } else {
          sample4(t)
        }
      }
    } else {
      return if (ENTER_COAST) {
        if (t <= t3f) {
          sample3(t)
        } else {
          sample4(t)
        }
      } else {
        sample4(t)
      }
    }
  }
}
