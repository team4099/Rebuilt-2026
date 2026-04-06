package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.AgitateIntakeCommand
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class IntakeQuadrantL1(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val intake: Intake,
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
                    interruptAtTimeout = false,
                    extraTime = .25.seconds),
                SequentialCommandGroup(
                    WaitCommand(0.75),
                    superstructure.requestIntakeCommand(),
                    WaitCommand(5.25),
                    superstructure.requestPrepScoreCommand(),
                    AimOTFCommand(drivetrain, timeout = 2.0.seconds),
                    superstructure.requestScoreCommand(),
                    WaitCommand(1.5),
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
                    AgitateIntakeCommand(superstructure, intake).withTimeout(6.5)))),
        superstructure.requestIdleCommand(),
        ParallelCommandGroup(
            FollowChoreoPath(
                drivetrain,
                secondSwipeTraj,
                flipVertically = flipVeritcally,
                interruptAtTimeout = false),
            WaitCommand(2.0)
                .andThen(
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_DOWN_ANGLE))))
  }

  companion object {
    val mainTraj =
        Choreo.loadTrajectory<SwerveSample>("IntakeQuadrantL1/IntakeQuadrantClimbQuick.traj").get()
    val secondSwipeTraj =
        Choreo.loadTrajectory<SwerveSample>("IntakeQuadrantL1/BackToQuadrant.traj").get()

    val startingPose = Pose2d(mainTraj.getInitialPose(false).get())
  }
}
