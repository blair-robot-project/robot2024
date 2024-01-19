package frc.team449.control.auto

import edu.wpi.first.wpilibj2.command.Command

/**
 * Interface to guide you as to how to create a Choreo routine class.
 */
interface ChoreoRoutineStructure {

  val routine: ChoreoRoutine

  val trajectory: MutableList<ChoreoTrajectory>

  /**
   * Method for returning the command of the entire auto routine
   * @see ChoreoRoutine.createRoutine
   */
  fun createCommand(): Command {
    return routine.createRoutine(trajectory)
  }
}
