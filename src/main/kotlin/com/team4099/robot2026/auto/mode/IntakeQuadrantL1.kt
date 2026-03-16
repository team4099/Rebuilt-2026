package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class IntakeQuadrantL1(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    flipVeritcally: Boolean
) : SequentialCommandGroup() {
  init {
    addCommands(
        SequentialCommandGroup(
            ParallelCommandGroup(
                FollowChoreoPath(
                    drivetrain,
                    mainTraj,
                    flipVertically = flipVeritcally,
                    interruptAtTimeout = true,
                    extraTime = 0.seconds),
                SequentialCommandGroup(
                    WaitCommand(1.0),
                    superstructure.requestIntakeCommand(),
                    WaitCommand(2.5),
                    superstructure.requestIdleCommand(),
                    WaitCommand(4.5),
                    superstructure.requestPrepScoreCommand())),
            AimOTFCommand(drivetrain, 6.seconds),
            superstructure.requestScoreCommand(),
            WaitCommand(1.5),
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
            WaitCommand(4.0),
            superstructure.requestIntakeAndScoreCommand()))
  }

  companion object {
    val mainTraj =
        Choreo.loadTrajectory<SwerveSample>("IntakeQuadrantL1/IntakeQuadrantClimb.traj").get()

    val startingPose = Pose2d(mainTraj.getInitialPose(false).get())
  }
}
