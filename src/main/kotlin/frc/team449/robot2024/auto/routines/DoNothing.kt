package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class DoNothing(
  private val robot: Robot
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive
    )

  override val trajectory: MutableList<ChoreoTrajectory> = mutableListOf()

  override fun createCommand(): Command {
    return AutoUtil.autoShoot(robot).andThen(
      robot.shooter.coast(),
      robot.undertaker.stop(),
      robot.feeder.stop()
    )
  }
}
