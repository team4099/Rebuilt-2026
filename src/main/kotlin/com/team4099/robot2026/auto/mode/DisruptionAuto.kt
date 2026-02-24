package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.RobotContainer.superstructure
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d

class DisruptionAuto(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, firstTrajectory),
            WaitCommand(.7)
                .andThen(
                    superstructure.requestForceIntakeDownCommand(),
                    WaitCommand(.97).andThen(superstructure.requestIntakeCommand())),
            WaitCommand(6.82).andThen(superstructure.requestIdleCommand())))
  }

  companion object {
    val firstTrajectory = Choreo.loadTrajectory<SwerveSample>("disruption/NewPath.traj").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
  }
}
