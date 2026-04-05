package com.team4099.robot2026.commands

import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class AgitateIntakeCommand(val superstructure: Superstructure) :
    SequentialCommandGroup() { // Used in auto
  init {
    addRequirements(superstructure)
    addCommands(
        RepeatCommand(
                SequentialCommandGroup(
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
                    WaitCommand(0.2),
                    superstructure.requestForceIntakeCommand(IntakeConstants.PIVOT_MIN_ANGLE),
                    WaitCommand(0.2)))
            .withTimeout(9.0),
        RepeatCommand(
            SequentialCommandGroup(
                superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
                WaitCommand(0.2),
                superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
                WaitCommand(0.2))))
  }
}
