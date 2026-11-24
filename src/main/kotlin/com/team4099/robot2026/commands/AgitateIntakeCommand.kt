package com.team4099.robot2026.commands

import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.max
import org.wpilib.command2.Commands
import org.wpilib.command2.RepeatCommand
import org.wpilib.command2.SequentialCommandGroup
import org.wpilib.command2.WaitCommand
import org.wpilib.command2.WaitUntilCommand

class AgitateIntakeCommand(val superstructure: Superstructure, val intake: Intake) :
    SequentialCommandGroup() {
  var lastStuckPosition = IntakeConstants.ANGLES.INTAKE_ANGLE

  init {
    addRequirements(superstructure)

    addCommands(
        RepeatCommand(
            SequentialCommandGroup(
                superstructure.requestForceIntakeCommand(
                    IntakeConstants.ANGLES.AUTO_AGITATE_FULL_UP),
                WaitUntilCommand {
                      intake.inputs.intakeStatorCurrent.absoluteValue >=
                          IntakeConstants.STATOR_CURRENT_LIMIT
                    }
                    .withTimeout(.2),
                Commands.runOnce({ lastStuckPosition = intake.inputs.position }),
                Commands.defer(
                    {
                      superstructure.requestForceIntakeCommand(
                          max(IntakeConstants.PIVOT_MIN_ANGLE, lastStuckPosition - 40.degrees))
                    },
                    setOf()),
                WaitCommand(.3))))
  }
}
