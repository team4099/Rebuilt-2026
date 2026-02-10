package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.RobotContainer.superstructure
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband

class CenterShootClimb(val drivetrain: Drive, val shooter: Shooter) : ParallelRaceGroup()  {
    init {
        addRequirements(drivetrain)

        addCommands(
            WaitCommand(.5).andThen(
                FollowChoreoPath(drivetrain, traj)),
            SequentialCommandGroup(WaitCommand(1.0).andThen(
                superstructure.requestIntakeCommand(),
                ),
                WaitCommand(1.5).andThen(
                    superstructure.requestForceIntakeUpCommand(),
                ),
                WaitCommand(3.0).andThen(
                    superstructure.requestPrepScoreCommand()
                ),
                WaitCommand(3.25).andThen(
                    AimOTFCommand(
                        drivetrain,
                        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                        { false },
                        Jessika()
                    )
                ),
                WaitUntilCommand { shooter.isAtTargetedVelocity }.andThen(
                    superstructure.requestScoreCommand(),
                )
            )
        )
    }

    companion object {
        val traj =
            Choreo.loadTrajectory<SwerveSample>("CenterInt_Shoot_Climb/path.chor").get()
        val startPose = Pose2d(traj.getInitialPose(false).get())
    }
}