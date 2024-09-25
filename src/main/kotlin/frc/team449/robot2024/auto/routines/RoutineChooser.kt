package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
      "Red3PieceMid" to Vision3PieceMid(robot, true).createCommand(),
      "Blue3PieceMid" to Vision3PieceMid(robot, false).createCommand(),
      "RedSubwoofer5Piece" to FivePieceSubwooferMiddy(robot, true).createCommand(),
      "BlueSubwoofer5Piece" to FivePieceSubwooferMiddy(robot, false).createCommand(),
      "RedSubwoofer5PieceFarthy" to FivePieceSubwooferFarthy(robot, true).createCommand(),
      "BlueSubwoofer5PieceFarthy" to FivePieceSubwooferFarthy(robot, false).createCommand(),
      "RedSubwoofer5PieceCenty" to FiveCentyFirst(robot, true).createCommand(),
      "BlueSubwoofer5PieceCenty" to FiveCentyFirst(robot, false).createCommand(),
      "RedSixPiece" to SixPiece(robot, true).createCommand(),
      "BlueSixPiece" to SixPiece(robot, false).createCommand(),
      "RedFourPieceHelper" to FourPieceHelper(robot, true).createCommand(),
      "BlueFourPieceHelper" to FourPieceHelper(robot, false).createCommand(),
      "RedThreePieceAmp" to ThreePieceAmpHelper(robot, true).createCommand(),
      "BlueThreePieceAmp" to ThreePieceAmpHelper(robot, false).createCommand(),
      "Red4" to FourPiece(robot, true).createCommand(),
      "Blue4" to FourPiece(robot, false).createCommand()
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption(
      "4 piece",
      "4"
    )

    this.addOption(
      "5 Piece Sub MIDDY",
      "Subwoofer5Piece"
    )

    this.addOption(
      "5 Piece Sub CENTY First",
      "Subwoofer5PieceCenty"
    )

    this.addOption(
      "5 Piece Sub FARTHY",
      "Subwoofer5PieceFarthy"
    )

    this.addOption(
      "3 Piece Mid",
      "3PieceMid"
    )

    this.addOption(
      "6 Piece",
      "SixPiece"
    )

    this.addOption(
      "4 Piece Helper Centerline",
      "FourPieceHelper"
    )

    this.addOption(
      "3 Piece Helper Amp Side",
      "ThreePieceAmp"
    )
  }
}
