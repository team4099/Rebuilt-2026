package com.team4099.robot2026.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.config.constants.ClimbConstants
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FeederConstants
import com.team4099.robot2026.config.constants.HopperConstants
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.config.constants.RollersConstants
import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Request.SuperstructureRequest
import com.team4099.robot2026.subsystems.superstructure.climb.Climb
import com.team4099.robot2026.subsystems.superstructure.feeder.Feeder
import com.team4099.robot2026.subsystems.superstructure.hopper.Hopper
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollers
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.geometry.Rotation2dWPILIB
import org.team4099.lib.geometry.Rotation3dWPILIB
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds

class Superstructure(
    private val drivetrain: Drive,
    private val vision: Vision,
    private val climb: Climb,
    private val feeder: Feeder,
    private val hopper: Hopper,
    private val intake: Intake,
    private val intakeRollers: IntakeRollers,
    private val shooter: Shooter
) : SubsystemBase() {
  var currentState: SuperstructureStates = SuperstructureStates.UNINITALIZED
    private set

  var currentRequest: SuperstructureRequest = SuperstructureRequest.Idle()
  var lastTransitionTime: Time = Clock.fpgaTime
    private set

  val field = Field2d()

  init {
    SmartDashboard.putData("Field", field)
  }

  override fun periodic() {
    val startTime = Clock.fpgaTime

    val climbStartTime = Clock.fpgaTime
    climb.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/ClimbLoopTimeMS", (Clock.fpgaTime - climbStartTime).inMilliseconds)

    val feederStartTime = Clock.fpgaTime
    feeder.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/FeederLoopTimeMS",
        (Clock.fpgaTime - feederStartTime).inMilliseconds)

    val hopperStartTime = Clock.fpgaTime
    hopper.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/HopperLoopTimeMS",
        (Clock.fpgaTime - hopperStartTime).inMilliseconds)

    val intakeStartTime = Clock.fpgaTime
    intake.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/IntakeLoopTimeMS",
        (Clock.fpgaTime - intakeStartTime).inMilliseconds)

    val intakeRollersStartTime = Clock.fpgaTime
    intakeRollers.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/IntakeRollersLoopTimeMS",
        (Clock.fpgaTime - intakeRollersStartTime).inMilliseconds)

    val shooterStartTime = Clock.fpgaTime
    shooter.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/ShooterLoopTimeMS",
        (Clock.fpgaTime - shooterStartTime).inMilliseconds)

    field.robotPose = drivetrain.pose.toPose2d().pose2d
    field.getObject("FUEL").setPoses(vision.objectsDetected[0].map{ Pose2dWPILIB(it.x.inMeters, it.y.inMeters,
      Rotation2dWPILIB())})

    var nextState = currentState

    CustomLogger.recordOutput("Superstructure/currentState", currentState)
    CustomLogger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)

    when (currentState) {
      SuperstructureStates.UNINITALIZED -> {
        nextState =
            if (Constants.Tuning.TUNING_MODE) SuperstructureStates.TUNING
            else SuperstructureStates.IDLE
      }
      SuperstructureStates.TUNING -> {
        // do nothing
      }
      SuperstructureStates.IDLE -> {
        climb.currentRequest =
            Request.ClimbRequest.TargetingPosition(ClimbConstants.DOWNWARDS_EXTENSION_LIMIT)
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.STOW_ANGLE)
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.IDLE_VOLTAGE)
        hopper.currentRequest = Request.HopperRequest.Idle()
        shooter.currentRequest = Request.ShooterRequest.Idle()

        nextState =
            when (currentRequest) {
              is SuperstructureRequest.ExtendClimb -> SuperstructureStates.PREP_CLIMB
              is SuperstructureRequest.PrepScore -> SuperstructureStates.PREP_SCORE
              is SuperstructureRequest.Intake -> SuperstructureStates.INTAKE
              is SuperstructureRequest.Eject -> SuperstructureStates.EJECT
              else -> currentState
            }
      }
      SuperstructureStates.PREP_SCORE -> {
        shooter.currentRequest =
            Request.ShooterRequest.TargetVelocity(ShooterConstants.VELOCITIES.SCORING_VELOCITY)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.Score -> {
            if (shooter.isAtTargetedVelocity) {
              nextState = SuperstructureStates.SCORE
            }
          }
          else -> {}
        }
      }
      SuperstructureStates.SCORE -> {
        feeder.currentRequest = Request.FeederRequest.OpenLoop(FeederConstants.SCORE_VOLTAGE)
        hopper.currentRequest =
            Request.HopperRequest.OpenLoop(HopperConstants.Voltages.SCORE_VOLTAGE)
        shooter.currentRequest =
            Request.ShooterRequest.TargetVelocity(ShooterConstants.VELOCITIES.SCORING_VELOCITY)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          else -> {}
        }
      }
      SuperstructureStates.INTAKE -> {
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.INTAKE_VOLTAGE)
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.INTAKE_ANGLE)

        if (currentRequest is SuperstructureRequest.Idle ||
            intakeRollers.inputs.rollerStatorCurrent > RollersConstants.FUEL_STALL_CURRENT &&
                (Clock.fpgaTime - lastTransitionTime) > RollersConstants.FUEL_STALL_TIME_THRESHOLD)
            nextState = SuperstructureStates.IDLE
      }
      SuperstructureStates.PREP_CLIMB -> {
        climb.currentRequest =
            Request.ClimbRequest.TargetingPosition(ClimbConstants.UPWARDS_EXTENSION_LIMIT)

        if (climb.isAtTargetedPosition) {
          nextState =
              when (currentRequest) {
                is SuperstructureRequest.Idle -> SuperstructureStates.IDLE
                is SuperstructureRequest.RetractClimb -> SuperstructureStates.CLIMB
                else -> currentState
              }
        }
      }
      SuperstructureStates.CLIMB -> {
        climb.currentRequest =
            Request.ClimbRequest.TargetingPosition(ClimbConstants.DOWNWARDS_EXTENSION_LIMIT)
        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.PREP_CLIMB
          else -> {}
        }
      }
      SuperstructureStates.EJECT -> {
        // If intake pivot is above EJECT_ANGLE, we can consider it to be stowed enough to be unable
        // to eject
        if (intake.inputs.position > IntakeConstants.ANGLES.EJECT_ANGLE) {
          intake.currentRequest =
              Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.INTAKE_ANGLE)
        }

        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.EJECT_VOLTAGE)
      }
    }

    if (currentState != nextState) lastTransitionTime = Clock.fpgaTime

    currentState = nextState
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/SuperstructureLoopTimeMS",
        (Clock.fpgaTime - startTime).inMilliseconds)
  }

  fun requestIdleCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.Idle() }
    returnCommand.name = "RequestIdleCommand"
    return returnCommand
  }

  fun requestIntakeCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.Intake() }
    returnCommand.name = "RequestIntakeCommand"
    return returnCommand
  }

  fun requestPrepScoreCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.PrepScore() }
    returnCommand.name = "RequestPrepScoreCommand"
    return returnCommand
  }

  fun requestScoreCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.Score() }
    returnCommand.name = "RequestScoreCommand"
    return returnCommand
  }

  fun requestPrepClimbCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.ExtendClimb() }
    returnCommand.name = "RequestPrepClimbCommand"
    return returnCommand
  }

  fun requestClimbCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.RetractClimb() }
    returnCommand.name = "RequestClimbCommand"
    return returnCommand
  }

  fun requestForceIntakeDownCommand(): Command {
    val returnCommand = runOnce {
      intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.INTAKE_ANGLE)
    }
    returnCommand.name = "RequestForceIntakeDownCommand"
    return returnCommand
  }

  fun requestForceIntakeUpCommand(): Command {
    val returnCommand = runOnce {
      intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.STOW_ANGLE)
    }
    returnCommand.name = "RequestForceIntakeUpCommand"
    return returnCommand
  }

  fun requestEjectCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.Eject() }
    returnCommand.name = "RequestEjectCommand"
    return returnCommand
  }

  companion object {
    enum class SuperstructureStates {
      UNINITALIZED,
      TUNING,
      IDLE,
      PREP_SCORE,
      SCORE,
      INTAKE,
      PREP_CLIMB,
      CLIMB,
      EJECT
    }
  }
}
