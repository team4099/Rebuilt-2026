package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d

class IntakeQuadrantL1(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    flipVeritcally: Boolean
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)
    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(drivetrain, traj, flipVertically = flipVeritcally),
            SequentialCommandGroup(
                WaitCommand(.5),
                superstructure.requestIntakeCommand(),
                WaitCommand(4.0),
                superstructure.requestIdleCommand())),
        superstructure.requestScoreCommand(),
        WaitCommand(2.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_UP_ANGLE),
        WaitCommand(1.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
        WaitCommand(1.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_UP_ANGLE),
        WaitCommand(1.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
        WaitCommand(1.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_UP_ANGLE),
        WaitCommand(1.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
        WaitCommand(1.0),
        superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_UP_ANGLE),
        WaitCommand(1.0),
        superstructure.requestIdleCommand(),
        WaitCommand(1.0),
        // the path ends near the tower
        // superstructure.requestPrepClimbCommand(),
        // superstructure.requestClimbCommand()
    )
  }

  companion object {
    val traj =
        Choreo.loadTrajectory<SwerveSample>("IntakeQuadrantL1/IntakeQuadrantClimb.traj").get()

    val startingPose = Pose2d(traj.getInitialPose(false).get())
  }
}
