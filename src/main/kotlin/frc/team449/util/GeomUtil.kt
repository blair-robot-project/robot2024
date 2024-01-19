package frc.team449.util

import edu.wpi.first.math.geometry.Twist2d




class GeomUtil {
  companion object {
    fun multiplyTwist(twist: Twist2d, factor: Double): Twist2d {
      return Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)
    }
  }
}