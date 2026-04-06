package com.team4099.robot2026.commands

import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.max

class AgitateIntakeCommand(val superstructure: Superstructure, val intake: Intake) :
    SequentialCommandGroup() {
  var lastStuckPosition = IntakeConstants.ANGLES.INTAKE_ANGLE

  init {
    addRequirements(superstructure)
    //    addCommands(
    //        RepeatCommand(
    //                SequentialCommandGroup(
    //                    superstructure.requestForceIntakeCommand(
    //                        IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
    //                    WaitCommand(0.2),
    //                    superstructure.requestForceIntakeCommand(IntakeConstants.PIVOT_MIN_ANGLE +
    // 5.degrees),
    //                    WaitCommand(0.2)))
    //            .withTimeout(9.0),
    //        RepeatCommand(
    //            SequentialCommandGroup(
    //
    // superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
    //                WaitCommand(0.2),
    //
    // superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
    //                WaitCommand(0.2))))

    addCommands(
        RepeatCommand(
            SequentialCommandGroup(
                superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.AUTO_AGITATE_FULL_UP),
                WaitUntilCommand {
                      intake.inputs.intakeStatorCurrent.absoluteValue >=
                          IntakeConstants.AGITATION_STUCK_STATOR_THRESHOLD
                    }
                    .withTimeout(.25),
                Commands.runOnce({ lastStuckPosition = intake.inputs.position }),
                Commands.defer(
                    {
                      superstructure.requestForceIntakeCommand(
                          max(IntakeConstants.PIVOT_MIN_ANGLE, lastStuckPosition - 40.degrees))
                    },
                    setOf()),
                WaitCommand(.4))))
  }
}
