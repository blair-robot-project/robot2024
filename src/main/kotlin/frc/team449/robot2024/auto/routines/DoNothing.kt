package frc.team449.robot2024.auto.routines

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

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
      ConditionalCommand(
        InstantCommand({
          robot.drive.pose = Pose2d(
            0.8079648017883301,
            6.623755931854248,
            Rotation2d(1.042722037034243)
          )
        }),
        InstantCommand({
          robot.drive.pose = Pose2d(
            FieldConstants.fieldLength - 0.8079648017883301,
            6.623755931854248,
            Rotation2d(PI - 1.042722037034243)
          )
        })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue },
      robot.shooter.coast(),
      robot.undertaker.stop(),
      robot.feeder.stop(),
    )
  }
}
