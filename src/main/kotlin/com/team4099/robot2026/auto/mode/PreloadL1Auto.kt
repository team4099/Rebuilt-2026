package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.DrivePathOTF
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d

class PreloadL1Auto(val drivetrain: Drive, val superstructure: Superstructure) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, firstTrajectory),
            superstructure.requestPrepScoreCommand()),
        ParallelCommandGroup(WaitCommand(0.5), superstructure.requestScoreCommand()),
        WaitCommand(5.0),
        superstructure.requestIdleCommand(),
        //      superstructure.requestPrepClimbCommand(),
        WaitCommand(2.0),
        DrivePathOTF.alignClimbTop(drivetrain),
        //      superstructure.requestClimbCommand()
    )
  }

  companion object {
    val firstTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/preloadShoot.traj").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
  }
}
