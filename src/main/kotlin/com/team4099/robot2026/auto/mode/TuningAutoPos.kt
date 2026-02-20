package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d

class TuningAutoPos(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      FollowChoreoPath(drivetrain, traj)
    )
  }

  companion object {
    val traj = Choreo.loadTrajectory<SwerveSample>("Test/path.traj").get()

    val startingPose = Pose2d(traj.getInitialPose(false).get())
  }
}