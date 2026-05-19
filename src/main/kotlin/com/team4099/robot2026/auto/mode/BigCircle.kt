package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d

class BigCircle(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        FollowChoreoPath(drivetrain, firstTrajectory),
    )
  }

  companion object {
    val firstTrajectory = Choreo.loadTrajectory<SwerveSample>("Tuning/NewPath.traj").get()

    // don't flip pose: poses are robot relative since field frame estimator was reset
    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
  }
}
