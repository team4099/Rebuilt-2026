package com.team4099.robot2026.auto.mode

import bline.commands.BLineCommandsV2 as BLineCommands
import bline.commands.FollowPathV2
import bline.path.Path
import com.team4099.robot2026.commands.AgitateIntakeCommand
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import org.wpilib.command2.SequentialCommandGroup
import org.wpilib.command2.WaitCommand
import org.wpilib.command2.WrapperCommand

class IntakeQuadrantFollowFar(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val intake: Intake,
    pathBuilder: (Path) -> FollowPathV2
) :
    WrapperCommand(
        BLineCommands.sequence(
            pathBuilder(pathOne).withPoseReset(),
            SequentialCommandGroup(
                WaitCommand(1.0),
                AgitateIntakeCommand(superstructure, intake).withTimeout(18.0 - (7.46 + 1.0)),
                superstructure.requestForceIntakeCommand(
                    IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE)))) {
  companion object {
    val pathOne = Path("intakequadfollowfar")
  }
}
