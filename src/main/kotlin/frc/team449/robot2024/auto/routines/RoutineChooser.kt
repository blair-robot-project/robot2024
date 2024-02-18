package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
      "Red4Piece" to FourPieceSubwoofer(robot, true).createCommand(),
      "Blue4Piece" to FourPieceSubwoofer(robot, false).createCommand(),
      "Red3PieceMid" to Experimental3PieceMid(robot, true).createCommand(),
      "Blue3PieceMid" to Experimental3PieceMid(robot, false).createCommand(),
      "RedSubwoofer5Piece" to FivePieceSubwoofer(robot, true).createCommand(),
      "BlueSubwoofer5Piece" to FivePieceSubwoofer(robot, false).createCommand(),
    )
  }

  init {
    updateOptions(true)
  }

  fun updateOptions(isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption(
      "4 Piece Subwoofer",
      if (isRed) "Red4Piece" else "Blue4Piece"
    )

    this.addOption(
      "5 Piece Subwoofer",
      if (isRed) "RedSubwoofer5Piece" else "BlueSubwoofer5Piece"
    )

    this.addOption(
      "3 Piece Mid",
      if (isRed) "Red3PieceMid" else "Blue3PieceMid"
    )
  }
}
