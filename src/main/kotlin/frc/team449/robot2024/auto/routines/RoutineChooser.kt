package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
      "Red4PieceAmp" to FourPieceSubwooferAmp(robot, true).createCommand(),
      "Blue4PieceAmp" to FourPieceSubwooferAmp(robot, false).createCommand(),
      "Red4PieceStage" to FourPieceSubwooferStage(robot, true).createCommand(),
      "Blue4PieceStage" to FourPieceSubwooferStage(robot, false).createCommand(),
      "Red3PieceMid" to Experimental3PieceMid(robot, true).createCommand(),
      "Blue3PieceMid" to Experimental3PieceMid(robot, false).createCommand(),
      "RedSubwoofer5Piece" to FivePieceSubwooferMiddy(robot, true).createCommand(),
      "BlueSubwoofer5Piece" to FivePieceSubwooferMiddy(robot, false).createCommand(),
      "RedSubwoofer5PieceFar" to FivePieceSubwooferFarFirst(robot, true).createCommand(),
      "BlueSubwoofer5PieceFar" to FivePieceSubwooferFarFirst(robot, false).createCommand(),
      "RedSubwoofer5PieceFarthy" to FivePieceSubwooferFarthy(robot, true).createCommand(),
      "BlueSubwoofer5PieceFarthy" to FivePieceSubwooferFarthy(robot, false).createCommand(),
    )
  }

  init {
    updateOptions(true)
  }

  fun updateOptions(isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption(
      "4 Piece Subwoofer Amp Side",
      if (isRed) "Red4PieceAmp" else "Blue4PieceAmp"
    )

    this.addOption(
      "4 Piece Subwoofer Stage Side",
      if (isRed) "Red4PieceStage" else "Blue4PieceStage"
    )

    this.addOption(
      "5 Piece Subwoofer MIDDY",
      if (isRed) "RedSubwoofer5Piece" else "BlueSubwoofer5Piece"
    )

    this.addOption(
      "5 Piece Subwoofer FARTHY",
      if (isRed) "RedSubwoofer5PieceFarthy" else "BlueSubwoofer5PieceFarthy"
    )

    this.addOption(
      "5 Piece Subwoofer FAR FIRST",
      if (isRed) "RedSubwoofer5PieceFar" else "BlueSubwoofer5PieceFar"
    )

    this.addOption(
      "3 Piece Mid",
      if (isRed) "Red3PieceMid" else "Blue3PieceMid"
    )
  }
}
