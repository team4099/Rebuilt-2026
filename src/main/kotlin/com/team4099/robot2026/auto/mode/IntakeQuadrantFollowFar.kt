package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.commands.AgitateIntakeCommand
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.robot.lib.BLine.BLineCommands
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path

class IntakeQuadrantFollowFar(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val intake: Intake,
    pathBuilder: FollowPath.Builder
) :
    WrapperCommand(
        BLineCommands.sequence(
            pathBuilder.build(pathOne),
            SequentialCommandGroup(
                WaitCommand(1.0),
                AgitateIntakeCommand(superstructure, intake).withTimeout(18.0 - (6.75 + 1.0)),
                superstructure.requestForceIntakeCommand(
                    IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE)))) {
  companion object {
    val pathOne = Path("intakequadfollowfar")
  }
}
