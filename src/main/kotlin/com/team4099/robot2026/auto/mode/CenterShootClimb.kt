package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.RobotContainer.superstructure
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.DrivePathOTF.Companion.alignClimbTop
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.util.AllianceFlipUtil
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.seconds

class CenterShootClimb(val drivetrain: Drive, val shooter: Shooter) : SequentialCommandGroup() {
  init {
    //    addRequirements(drivetrain)

    addCommands(
        Commands.runOnce({ drivetrain.pose = Pose3d(AllianceFlipUtil.apply(startPose)) }),
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, traj),
            SequentialCommandGroup(
                superstructure.requestIdleCommand(),
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
                    WaitUntilCommand { shooter.isAtTargetedVelocity }
                        .andThen(
                            superstructure.requestScoreCommand(),
                        )),
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
