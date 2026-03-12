package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class TestingAuto(val drivetrain: Drive, val superstructure: Superstructure) :
    SequentialCommandGroup() {
  init {
    addCommands(
        superstructure.requestScoreCommand(),
        RepeatCommand(
                SequentialCommandGroup(
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
                    WaitCommand(0.1),
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
                    WaitCommand(0.1)))
            .withTimeout(4.5),
        RepeatCommand(
            SequentialCommandGroup(
                superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_UP_ANGLE),
                WaitCommand(0.1),
                superstructure.requestForceIntakeCommand(
                    IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
                WaitCommand(0.1))))
  }
}
