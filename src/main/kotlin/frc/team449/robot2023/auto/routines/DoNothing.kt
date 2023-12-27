package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot

class DoNothing(
  robot: Robot
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive
    )

  override val trajectory: MutableList<ChoreoTrajectory> = mutableListOf()

  override fun createCommand(): Command {
    return InstantCommand()
  }
}
