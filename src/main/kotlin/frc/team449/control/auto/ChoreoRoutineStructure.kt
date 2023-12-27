package frc.team449.control.auto

import edu.wpi.first.wpilibj2.command.Command

interface ChoreoRoutineStructure {

  val routine: ChoreoRoutine

  val trajectory: MutableList<ChoreoTrajectory>

  fun createCommand(): Command {
    return routine.createRoutine(trajectory)
  }
}
