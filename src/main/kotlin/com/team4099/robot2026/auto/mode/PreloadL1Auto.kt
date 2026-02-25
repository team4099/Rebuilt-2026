package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
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
        FollowChoreoPath(drivetrain, firstTrajectory),
        ParallelCommandGroup(WaitCommand(.4), superstructure.requestScoreCommand()),
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, secondTrajectory),
            superstructure.requestPrepClimbCommand()),
        superstructure.requestClimbCommand())
  }

  companion object {
    val firstTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/preloadShoot.traj").get()
    // prep shoot

    val secondTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/climb.traj").get()

    // shoot
    // don't flip pose: poses are robot relative since field frame estimator was reset
    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
  }

  class PreloadL1Auto(val drivetrain: Drive, val superstructure: Superstructure) :
      SequentialCommandGroup() {
    init {
      addRequirements(drivetrain)

      addCommands(
          FollowChoreoPath(drivetrain, firstTrajectory),
          ParallelCommandGroup(WaitCommand(2.0), superstructure.requestScoreCommand()),
          ParallelCommandGroup(
              FollowChoreoPath(drivetrain, secondTrajectory),
              superstructure.requestPrepClimbCommand()),
          superstructure.requestClimbCommand())
    }

    companion object {
      val firstTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/preloadShoot.traj").get()
      // prep shoot

      val secondTrajectory = Choreo.loadTrajectory<SwerveSample>("preload/climb.traj").get()

      // shoot
      // don't flip pose: poses are robot relative since field frame estimator was reset
      val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
    }
  }
}
