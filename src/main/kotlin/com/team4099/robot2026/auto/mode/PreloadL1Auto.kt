package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class PreloadL1Auto(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val flipVertically: Boolean
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, firstTrajectory, flipVertically = flipVertically),
            superstructure.requestPrepScoreCommand()),
        ParallelCommandGroup(
            AimOTFCommand(drivetrain, 5.seconds),
            WaitCommand(0.5).andThen(superstructure.requestScoreCommand())).withTimeout(5.0),
        superstructure.requestIdleCommand(),
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, secondTrajectory, flipVertically = flipVertically),
            SequentialCommandGroup(
                WaitCommand(1.5),
                superstructure.requestIntakeCommand(),
                WaitCommand(4.0),
                superstructure.requestPrepScoreCommand(),
                ParallelCommandGroup(
                    AimOTFCommand(drivetrain, 10.seconds),
                    WaitCommand(0.5).andThen(superstructure.requestScoreCommand())))))
  }

  companion object {
    val firstTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/PreloadShoot.traj").get()
    val secondTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/GoToQuadrant.traj").get()
    // val secondTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/climb.traj").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
  }
}
