package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
      "Red3PieceMid" to Experimental3PieceMid(robot, true).createCommand(),
      "Blue3PieceMid" to Experimental3PieceMid(robot, false).createCommand(),
      "RedSubwoofer5Piece" to FivePieceSubwooferMiddy(robot, true).createCommand(),
      "BlueSubwoofer5Piece" to FivePieceSubwooferMiddy(robot, false).createCommand(),
      "RedSubwoofer5PieceFarthy" to FivePieceSubwooferFarthy(robot, true).createCommand(),
      "BlueSubwoofer5PieceFarthy" to FivePieceSubwooferFarthy(robot, false).createCommand(),
      "RedSubwoofer5PieceCenty" to FivePieceSubwooferCenty(robot, true).createCommand(),
      "BlueSubwoofer5PieceCenty" to FivePieceSubwooferCenty(robot, false).createCommand(),
      "RedSixPiece" to SixPiece(robot, true).createCommand(),
      "BlueSixPiece" to SixPiece(robot, false).createCommand(),
      "RedFourPieceHelper" to FourPieceHelper(robot, true).createCommand(),
      "BlueFourPieceHelper" to FourPieceHelper(robot, false).createCommand(),
    )
  }

  init {
    updateOptions(true)
  }

  fun updateOptions(isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption(
      "5 Piece Subwoofer MIDDY",
      if (isRed) "RedSubwoofer5Piece" else "BlueSubwoofer5Piece"
    )

    this.addOption(
      "5 Piece Subwoofer CENTY",
      if (isRed) "RedSubwoofer5PieceCenty" else "BlueSubwoofer5PieceCenty"
    )

    this.addOption(
      "5 Piece Subwoofer FARTHY",
      if (isRed) "RedSubwoofer5PieceFarthy" else "BlueSubwoofer5PieceFarthy"
    )

    this.addOption(
      "3 Piece Mid",
      if (isRed) "Red3PieceMid" else "Blue3PieceMid"
    )

    this.addOption(
      "6 Piece",
      if (isRed) "RedSixPiece" else "BlueSixPiece"
    )

    this.addOption(
      "4 Piece Helper Centerline",
      if (isRed) "RedFourPieceHelper" else "BlueFourPieceHelper"
    )
  }
}
