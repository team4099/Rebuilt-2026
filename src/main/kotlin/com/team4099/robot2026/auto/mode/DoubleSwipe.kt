package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.auto.mode.CenterlineSweep.Companion.mainTraj
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d

class DoubleSwipe(
    val drivetrain: Drive,
    val superstructure: Superstructure,
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        SequentialCommandGroup(
            ParallelCommandGroup(
                FollowChoreoPath(drivetrain, path2),
                SequentialCommandGroup(
                    WaitCommand(1.0),
                    superstructure.requestIntakeCommand(),
                    WaitCommand(3.0),
                ),
            ),
            superstructure.requestScoreCommand(),
            WaitCommand(8.0),
            ParallelCommandGroup(
                FollowChoreoPath(drivetrain, path1),
                SequentialCommandGroup(
                    WaitCommand(2.0),
                    superstructure.requestIntakeCommand(),
                ))))
  }

  companion object {
    val path1 = Choreo.loadTrajectory<SwerveSample>("DoubleSwipe/Path1.traj").get()
    val path2 = Choreo.loadTrajectory<SwerveSample>("DoubleSwipe/Path2.traj").get()
      val startingPose = Pose2d(path2.getInitialPose(false).get())
  }
}
