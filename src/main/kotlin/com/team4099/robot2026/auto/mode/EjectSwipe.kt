package com.team4099.robot2026.auto.mode

import bline.commands.BLineCommandsV2 as BLineCommands
import bline.path.Path
import com.team4099.robot2026.auto.AutoPathBuilder
import com.team4099.robot2026.commands.AgitateIntakeCommand
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import org.wpilib.command2.WrapperCommand

class EjectSwipe(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val intake: Intake,
    pathBuilder: AutoPathBuilder
) :
    WrapperCommand(
        BLineCommands.sequence(
            pathBuilder.build(pathOne),
            BLineCommands.sequence(
                pathBuilder.build(pathTwo, resetPose = false),
                AgitateIntakeCommand(superstructure, intake)
                    .withTimeout(18.0 - (8.94) - (7.30) - (1.0)),
                superstructure.requestForceIntakeCommand(
                    IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE)))) {

  companion object {
    val pathOne = Path("intakeeject")
    val pathTwo = Path("intakeshoot")
  }
}
