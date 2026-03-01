package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.auto.mode.IntakeQuadrantL1.Companion.climbFlippedTraj
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class CenterlineSweep(
  val drivetrain: Drive,
  val superstructure: Superstructure,
  flipVeritcally: Boolean
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)
    addCommands(
      SequentialCommandGroup(
        ParallelCommandGroup(
          FollowChoreoPath(drivetrain, mainTraj, flipVertically = flipVeritcally),
          SequentialCommandGroup(
            superstructure.requestIntakeCommand(),
            WaitCommand(3.0),
            superstructure.requestIdleCommand(),
            superstructure.requestPrepScoreCommand())
        ),
        superstructure.requestScoreCommand(),
        WaitUntilCommand({superstructure.currentState == Superstructure.Companion.SuperstructureStates.SCORE}),
        WaitCommand(1.0),
        RepeatCommand(
          SequentialCommandGroup(
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
            WaitCommand(0.1),
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
            WaitCommand(0.1)
          )
        ).withTimeout(4.0),
        RepeatCommand(
          SequentialCommandGroup(
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
            WaitCommand(0.1),
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
            WaitCommand(0.1)
          )
        ).withTimeout(4.0),
        superstructure.requestIdleCommand(),
        WaitCommand(1.0),
      ).withTimeout(12.0),
      superstructure.requestIdleCommand(),
      superstructure.requestPrepClimbCommand(),
      ConditionalCommand(
        FollowChoreoPath(drivetrain, climbFlippedTraj),
        InstantCommand()
      ) { flipVeritcally },
      superstructure.requestClimbCommand()
    )
  }

  companion object {
    val mainTraj =
      Choreo.loadTrajectory<SwerveSample>("SweepCenter/MidlineSweep.traj").get()

    val climbFlippedTraj = Choreo.loadTrajectory<SwerveSample>("SweepCenter/Climb.traj").get()

    val startingPose = Pose2d(mainTraj.getInitialPose(false).get())
  }
}
