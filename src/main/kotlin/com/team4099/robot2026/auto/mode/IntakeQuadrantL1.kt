package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class IntakeQuadrantL1(val drivetrain: Drive, val superstructure: Superstructure) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)
    addCommands(
        ParallelCommandGroup(
            FollowChoreoPath(
                drivetrain, Choreo.loadTrajectory<SwerveSample>("IntakeQuadrantClimb.traj").get()),
            WaitCommand(1.3).andThen(superstructure.requestIntakeCommand()),
            WaitCommand(2.5).andThen(superstructure.requestIdleCommand()),
        ),
        superstructure.requestScoreCommand(),
        WaitCommand(10.0),
        // the path ends near the tower
        // superstructure.requestPrepClimbCommand(),
        // superstructure.requestClimbCommand()
    )
  }
}
