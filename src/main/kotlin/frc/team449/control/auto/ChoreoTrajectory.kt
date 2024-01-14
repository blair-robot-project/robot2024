package frc.team449.control.auto

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Filesystem
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.io.File
import java.io.FileReader
import kotlin.math.abs

class ChoreoTrajectory(
  val name: String,
  val stateMap: InterpolatingMatrixTreeMap<Double, N2, N3>,
  val totalTime: Double,
  val objectiveTimestamps: ArrayList<Double>
) {

  fun initialPose(): Pose2d {
    val initialState = stateMap.get(0.0)

    return Pose2d(
      initialState[0, 0],
      initialState[0, 1],
      Rotation2d(initialState[0, 2])
    )
  }

  fun sample(t: Double): ChoreoState {
    val timeSeconds = MathUtil.clamp(t, 0.0, totalTime)
    val interpolatedMat = stateMap.get(timeSeconds)

    return ChoreoState(
      interpolatedMat[0, 0],
      interpolatedMat[0, 1],
      interpolatedMat[0, 2],
      interpolatedMat[1, 0],
      interpolatedMat[1, 1],
      interpolatedMat[1, 2]
    )
  }

  class ChoreoState(
    val xPos: Double,
    val yPos: Double,
    val theta: Double,
    val xVel: Double,
    val yVel: Double,
    val thetaVel: Double
  )

  companion object {
    fun createTrajectory(
      filename: String
    ): MutableList<ChoreoTrajectory> {
      val path = Filesystem.getDeployDirectory().absolutePath.plus("/choreo/$filename.chor")
      val document = (JSONParser().parse(FileReader(File(path).absolutePath)) as JSONObject)["paths"] as HashMap<*, *>

      val trajList = mutableListOf<ChoreoTrajectory>()

      document.forEach { (name, pathData) ->
        name as String
        pathData as JSONObject
        val trajectory = pathData["trajectory"] as JSONArray

        val info = parse(trajectory)

        val last = trajectory.last() as JSONObject
        val totalTime = last["timestamp"] as Double

        trajList.add(
          ChoreoTrajectory(
            filename + name,
            info.first,
            totalTime,
            info.second
          )
        )
      }

      return trajList
    }

    private fun deadband(value: Double, deadband: Double = 1e-6): Double {
      return if (abs(value) > deadband) value else 0.0
    }

    private fun parse(trajectory: JSONArray): Pair<InterpolatingMatrixTreeMap<Double, N2, N3>, ArrayList<Double>> {
      val stateMap = InterpolatingMatrixTreeMap<Double, N2, N3>()

      val timestamps = arrayListOf<Double>()

      trajectory.forEach { state ->
        state as JSONObject
        val stateTime = state["timestamp"].toString().toDouble()

        timestamps.add(stateTime)

        val matrix = MatBuilder.fill(
          Nat.N2(),
          Nat.N3(),
          deadband(state["x"].toString().toDouble()),
          deadband(state["y"].toString().toDouble()),
          deadband(state["heading"].toString().toDouble()),
          deadband(state["velocityX"].toString().toDouble()),
          deadband(state["velocityY"].toString().toDouble()),
          deadband(state["angularVelocity"].toString().toDouble())
        )

        stateMap.put(stateTime, matrix)
      }

      return stateMap to timestamps
    }
  }
}
