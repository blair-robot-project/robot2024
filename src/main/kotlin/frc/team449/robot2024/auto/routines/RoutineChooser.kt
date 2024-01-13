package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
      "Red4Piece" to Experimental4Piece(robot, true).createCommand(),
      "Blue4Piece" to Experimental4Piece(robot, false).createCommand()
    )
  }

  init {
    updateOptions(true)
  }

  fun updateOptions(isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption(
      "Experimental 4 Piece",
      if (isRed) {
        "Red4Piece"
      } else {
        "Blue4Piece"
      }
    )
  }
}
