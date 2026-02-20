package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class TestOTFAuto(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, traj, overrideRotationTrigger = { true }),
            AimOTFCommand(drivetrain, 5.0.seconds)))
  }

  companion object {
    val traj = Choreo.loadTrajectory<SwerveSample>("Test/path.traj").get()

    val startingPose = Pose2d(traj.getInitialPose(false).get())
  }
}
