package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.DrivePathOTF.Companion.alignClimbTop
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class CenterShootClimb(drivetrain: Drive, superstructure: Superstructure) :
    SequentialCommandGroup() {
  init {
    //    addRequirements(drivetrain)

    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, traj),
            SequentialCommandGroup(
                WaitCommand(1.0)
                    .andThen(
                        superstructure.requestIntakeCommand(),
                    ),
                WaitCommand(2.0)
                    .andThen(
                        superstructure.requestIdleCommand(),
                    ),
                WaitCommand(0.5).andThen(superstructure.requestPrepScoreCommand()),
                ParallelCommandGroup(
                    WaitCommand(1.0)
                        .andThen(
                            AimOTFCommand(
                                drivetrain,
                                7.0.seconds,
                            )),
                    superstructure.requestScoreCommand(),
                ),
                superstructure.requestIdleCommand(),
                superstructure.requestPrepClimbCommand(),
            )),
        alignClimbTop(drivetrain),
        superstructure.requestClimbCommand())
  }

  companion object {
    val traj =
        Choreo.loadTrajectory<SwerveSample>("CenterInt_Shoot_Climb/CenterIntakeMiddleClimb.traj")
            .get()
    val startPose = Pose2d(traj.getInitialPose(false).get())
  }
}
