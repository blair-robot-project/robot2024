package frc.team449.robot2024.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.math.PI

object AutoUtil {

  fun transformForRed(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder.fill(
          Nat.N2(),
          Nat.N3(),
          FieldConstants.fieldLength - currentMatrix[0, 0],
          currentMatrix[0, 1],
          MathUtil.angleModulus(PI - currentMatrix[0, 2]),
          -currentMatrix[1, 0],
          currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  /** Add other methods that return commands that do groups of actions that are done
   * across different auto routines. For Charged UP, these methods were things such as
   * dropping a cone/cube, or getting in ground intake position, etc.
   */
}
