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

class DepotL1OutpostScoreAuto (val drivetrain: Drive, val superstructure: Superstructure): SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, firstTrajectory),
        WaitCommand(1.0).andThen(superstructure.requestForceIntakeDownCommand()),
        WaitCommand(1.63).andThen(superstructure.requestIntakeCommand()),
        ), WaitCommand(2.58).andThen(superstructure.requestForceIntakeUpCommand()),
      WaitCommand(2.0),
      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, secondTrajectory),
        superstructure.requestPrepScoreCommand()
      ),
      ParallelCommandGroup(
        WaitCommand(9.0),
        superstructure.requestScoreCommand()
      ),
      ParallelCommandGroup(
      FollowChoreoPath(drivetrain,thirdTrajectory),
        superstructure.requestPrepClimbCommand()
        ),
      superstructure.requestClimbCommand())
  }
  companion object {
    val firstTrajectory =
      Choreo.loadTrajectory<SwerveSample>("DepotL1OutpostScore/DepotL1Outpost.traj").get()
    val secondTrajectory =
      Choreo.loadTrajectory<SwerveSample>("DepotL1OutpostScore/outpost/traj").get()
    val thirdTrajectory =
      Choreo.loadTrajectory<SwerveSample>("DepotL1OutpostScore/shootClimb.traj").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
  }
}