package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.seconds

class IntakeSideSpin(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    flipVeritcally: Boolean
) : SequentialCommandGroup() {
  init {
    addCommands(
        SequentialCommandGroup(
            ParallelCommandGroup(
                FollowChoreoPath(
                    drivetrain,
                    mainTraj,
                    flipVertically = flipVeritcally,
                    interruptAtTimeout = false,
                    extraTime = .25.seconds),
                SequentialCommandGroup(
                    WaitCommand(0.75),
                    superstructure.requestIntakeCommand(),
                    WaitCommand(8.5),
                    superstructure.requestPrepScoreCommand(),
                    AimOTFCommand(drivetrain, timeout = 1.0.seconds).until {
                      AimOTFCommand.hasAligned
                    },
                )),
            superstructure.requestScoreCommand(),
            WaitCommand(2.5),
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
            RepeatCommand(
                    SequentialCommandGroup(
                        superstructure.requestForceIntakeCommand(
                            IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
                        WaitCommand(0.1),
                        superstructure.requestForceIntakeCommand(
                            IntakeConstants.ANGLES.FORCE_DOWN_ANGLE),
                        WaitCommand(0.1)))
                .withTimeout(6.5),
            RepeatCommand(
                SequentialCommandGroup(
                    superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE),
                    WaitCommand(0.2),
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_HALFDOWN_ANGLE),
                    WaitCommand(0.2)))))
  }

  companion object {
    val mainTraj =
        Choreo.loadTrajectory<SwerveSample>("IntakeQuadrantL1/IntakeQuadrantSpin.traj").get()

    val startingPose = Pose2d(mainTraj.getInitialPose(false).get())
  }
}
