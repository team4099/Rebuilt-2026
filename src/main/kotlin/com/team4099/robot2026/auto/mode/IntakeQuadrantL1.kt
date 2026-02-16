package com.team4099.robot2026.auto.mode

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
            FollowChoreoPath(),
            WaitCommand(1.3).andThen(superstructure.requestIntakeCommand()),
            WaitCommand(2.5).andThen(superstructure.requestIntakeCommand()),
        ),
        superstructure.requestScoreCommand(),
        WaitCommand(10.0), // should i use a constant here
        // superstructure.requestPrepClimbCommand(),
        // superstructure.requestClimbCommand()
    )
  }
}
