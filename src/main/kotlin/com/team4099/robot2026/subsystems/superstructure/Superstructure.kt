package com.team4099.robot2026.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.RobotContainer
import com.team4099.robot2026.RobotContainer.intakeOverridingAngle
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
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
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.max

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
  var lastTransitionTime: Time = Clock.timestamp
    private set

  val launchData: Shooter.Companion.CalculatedLaunchData
    get() = Shooter.calculateLaunchData(drivetrain.pose.toPose2d(), drivetrain.chassisSpeeds)

  var overrideShooterVelocity = false
  var defenseMode = false

  val shooterTargetRPM: AngularVelocity
    get() {
      return if (overrideShooterVelocity) ShooterConstants.VELOCITIES.MANUAL_SHOOTING
      else
          max(
              Shooter.distanceToShooterRPM(launchData.distanceToTarget),
              ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY)
    }

  val field = Field2d()

  var jigglingIntake = false

  init {
    SmartDashboard.putData("Field", field)
  }

  override fun periodic() {
    val startTime = Clock.epochTime
    CustomLogger.recordOutput("idk", (intakeOverridingAngle + 10.degrees).inDegrees)

    val climbStartTime = Clock.epochTime
    climb.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/ClimbLoopTimeMS", (Clock.epochTime - climbStartTime).inMilliseconds)

    val feederStartTime = Clock.epochTime
    feeder.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/FeederLoopTimeMS",
        (Clock.epochTime - feederStartTime).inMilliseconds)

    val hopperStartTime = Clock.epochTime
    hopper.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/HopperLoopTimeMS",
        (Clock.epochTime - hopperStartTime).inMilliseconds)

    val intakeStartTime = Clock.epochTime
    intake.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/IntakeLoopTimeMS",
        (Clock.epochTime - intakeStartTime).inMilliseconds)

    val intakeRollersStartTime = Clock.epochTime
    intakeRollers.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/IntakeRollersLoopTimeMS",
        (Clock.epochTime - intakeRollersStartTime).inMilliseconds)

    val shooterStartTime = Clock.epochTime
    shooter.onLoop()
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/ShooterLoopTimeMS",
        (Clock.epochTime - shooterStartTime).inMilliseconds)

    field.robotPose = drivetrain.pose.toPose2d().pose2d

    var nextState = currentState

    CustomLogger.recordOutput("Superstructure/currentState", currentState)
    CustomLogger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Superstructure/overrideShooterVelocity", overrideShooterVelocity)
    CustomLogger.recordOutput("Superstructure/defenseMode", defenseMode)

    when (currentState) {
      SuperstructureStates.UNINITALIZED -> {
        nextState =
            if (Constants.Tuning.TUNING_MODE) SuperstructureStates.TUNING
            else SuperstructureStates.IDLE
      }
      SuperstructureStates.TUNING -> {
        if (currentRequest is SuperstructureRequest.Score) {
          shooter.currentRequest =
              Request.ShooterRequest.TargetVelocity(shooter.shooterTestVel.get())
          CustomLogger.recordOutput(
              "Superstructure/targetLaunchVelMPS", launchData.launchVelocity.inMetersPerSecond)

          if (shooter.isAtTargetedVelocity) {
            feeder.currentRequest = Request.FeederRequest.TargetVelocity(feeder.feederTestVel.get())
            hopper.currentRequest = Request.HopperRequest.TargetVelocity(hopper.hopperTestVel.get())
            intakeRollers.currentRequest =
                Request.RollersRequest.OpenLoop(RollersConstants.SCORE_ASSISTING_VOLTAGE)
          }
        } else {
          shooter.currentRequest =
              Request.ShooterRequest.TargetVelocity(ShooterConstants.VELOCITIES.IDLE_VELOCITY)
          feeder.currentRequest = Request.FeederRequest.Idle()
          hopper.currentRequest = Request.HopperRequest.Idle()
          intakeRollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersConstants.IDLE_VOLTAGE)
        }
      }
      SuperstructureStates.IDLE -> {
        climb.currentRequest =
            Request.ClimbRequest.TargetingPosition(ClimbConstants.DOWNWARDS_EXTENSION_LIMIT)
        feeder.currentRequest = Request.FeederRequest.Idle()
        hopper.currentRequest = Request.HopperRequest.Idle()
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(
                if (!defenseMode) IntakeConstants.ANGLES.IDLE_ANGLE
                else IntakeConstants.ANGLES.STOW_ANGLE)
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.IDLE_VOLTAGE)
        shooter.currentRequest = Request.ShooterRequest.Idle()

        intakeOverridingAngle = IntakeConstants.ANGLES.INTAKE_ANGLE

        nextState =
            when (currentRequest) {
              is SuperstructureRequest.ForceHome -> SuperstructureStates.FORCE_HOME
              is SuperstructureRequest.Unjam -> SuperstructureStates.UNJAM
              // is SuperstructureRequest.ExtendClimb -> SuperstructureStates.PREP_CLIMB
              is SuperstructureRequest.PrepScore -> SuperstructureStates.PREP_SCORE
              is SuperstructureRequest.Score -> SuperstructureStates.SCORE
              is SuperstructureRequest.Intake -> SuperstructureStates.INTAKE
              is SuperstructureRequest.Eject -> SuperstructureStates.EJECT
              else -> currentState
            }
      }
      SuperstructureStates.UNJAM -> {
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.PIVOT_MIN_ANGLE)
        hopper.currentRequest = Request.HopperRequest.OpenLoop(HopperConstants.UNJAM_VOLTAGE)
        feeder.currentRequest = Request.FeederRequest.OpenLoop(FeederConstants.UNJAM_VOLTAGE)

        when (currentRequest) {
          is SuperstructureRequest.Idle,
          is SuperstructureRequest.PrepScore,
          is SuperstructureRequest.Score,
          is SuperstructureRequest.Intake -> nextState = SuperstructureStates.IDLE
          else -> {}
        }
      }
      SuperstructureStates.FORCE_HOME -> {
        intake.currentRequest =
            Request.IntakeRequest.OpenLoop(IntakeConstants.FORCE_HOME_INTAKE_VOLTAGE)

        if (currentRequest is SuperstructureRequest.Idle ||
            currentRequest is SuperstructureRequest.Intake ||
            currentRequest is SuperstructureRequest.PrepScore ||
            currentRequest is SuperstructureRequest.Score)
            intake.currentRequest =
                Request.IntakeRequest.ZeroPivot(IntakeConstants.ANGLES.RESET_INTAKE_ANGLE)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.Intake -> nextState = SuperstructureStates.INTAKE
          is SuperstructureRequest.PrepScore -> nextState = SuperstructureStates.PREP_SCORE
          is SuperstructureRequest.Score -> nextState = SuperstructureStates.SCORE
          else -> {}
        }
      }
      SuperstructureStates.PREP_SCORE -> {
        shooter.currentRequest = Request.ShooterRequest.TargetVelocity(shooterTargetRPM)
        hopper.currentRequest = Request.HopperRequest.Idle()
        feeder.currentRequest = Request.FeederRequest.Idle()
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.IDLE_VOLTAGE)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.Intake -> nextState = SuperstructureStates.INTAKE
          // is SuperstructureRequest.ExtendClimb -> nextState = SuperstructureStates.PREP_CLIMB
          is SuperstructureRequest.Score -> {
            if (shooter.isAtTargetedVelocity) {
              nextState = SuperstructureStates.SCORE
            }
          }
          else -> {}
        }
      }
      SuperstructureStates.SCORE -> {
        shooter.currentRequest = Request.ShooterRequest.TargetVelocity(shooterTargetRPM)
        feeder.currentRequest = Request.FeederRequest.TargetVelocity(FeederConstants.SCORE_VELOCITY)

        if (shooter.isAtTargetedVelocity &&
            feeder.isAtTargetedVelocity &&
            (AimOTFCommand.hasAligned || !RobotContainer.isAligning || overrideShooterVelocity)) {
          hopper.currentRequest =
              Request.HopperRequest.TargetVelocity(HopperConstants.VELOCITIES.SCORE_VELOCITY)
          intakeRollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersConstants.SCORE_ASSISTING_VOLTAGE)
        }

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.Intake -> nextState = SuperstructureStates.IDLE
          // is SuperstructureRequest.ExtendClimb -> nextState = SuperstructureStates.PREP_CLIMB
          else -> {}
        }
      }
      SuperstructureStates.SCORE_AND_INTAKE -> {
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.INTAKE_ANGLE)
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.TELEOP_INTAKE_VOLTAGE)
        shooter.currentRequest = Request.ShooterRequest.TargetVelocity(shooterTargetRPM)

        if (shooter.isAtTargetedVelocity &&
            (AimOTFCommand.hasAligned || !RobotContainer.isAligning || overrideShooterVelocity)) {
          feeder.currentRequest = Request.FeederRequest.OpenLoop(FeederConstants.SCORE_VOLTAGE)
          hopper.currentRequest = Request.HopperRequest.OpenLoop(HopperConstants.SCORE_VOLTAGE)
        }

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          // is SuperstructureRequest.ExtendClimb -> nextState = SuperstructureStates.PREP_CLIMB
          else -> {}
        }
      }
      SuperstructureStates.INTAKE -> {
        shooter.currentRequest = Request.ShooterRequest.Idle()
        hopper.currentRequest = Request.HopperRequest.Idle()
        feeder.currentRequest = Request.FeederRequest.Idle()

        if (DriverStation.isAutonomous()) { // Run intake faster in teleop
          intakeRollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersConstants.AUTO_INTAKE_VOLTAGE)
        } else {
          intakeRollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersConstants.TELEOP_INTAKE_VOLTAGE)
        }

        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(
                if (jigglingIntake) IntakeConstants.ANGLES.INTAKING_JIGGLE_ANGLE
                else IntakeConstants.ANGLES.INTAKE_ANGLE)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.PrepScore -> nextState = SuperstructureStates.PREP_SCORE
          is SuperstructureRequest.Score -> nextState = SuperstructureStates.SCORE
          is SuperstructureRequest.Eject -> nextState = SuperstructureStates.EJECT
          // is SuperstructureRequest.ExtendClimb -> nextState = SuperstructureStates.PREP_CLIMB
          else -> {}
        }
      }
      SuperstructureStates.PREP_CLIMB -> {
        shooter.currentRequest = Request.ShooterRequest.Idle()
        hopper.currentRequest = Request.HopperRequest.Idle()
        feeder.currentRequest = Request.FeederRequest.Idle()
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.IDLE_VOLTAGE)
        climb.currentRequest =
            Request.ClimbRequest.TargetingPosition(ClimbConstants.PREP_CLIMB_HEIGHT)
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.CLIMB_ANGLE)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          //          is SuperstructureRequest.RetractClimb -> {
          //            if (climb.isAtTargetedPosition) {
          //              nextState = SuperstructureStates.CLIMB
          //            }
          //          }
          else -> {}
        }
      }
      SuperstructureStates.CLIMB -> {
        climb.currentRequest = Request.ClimbRequest.TargetingPosition(ClimbConstants.CLIMB_HEIGHT)
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.CLIMB_ANGLE)

        when (currentRequest) {
          // is SuperstructureRequest.ExtendClimb -> nextState = SuperstructureStates.PREP_CLIMB
          else -> {}
        }
      }
      SuperstructureStates.EJECT -> {
        intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(IntakeConstants.ANGLES.EJECT_ANGLE)
        intakeRollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersConstants.EJECT_VOLTAGE)

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.Intake -> nextState = SuperstructureStates.INTAKE
          // is SuperstructureRequest.ExtendClimb -> nextState = SuperstructureStates.PREP_CLIMB
          else -> {}
        }
      }
    }

    if (currentState != nextState) lastTransitionTime = Clock.timestamp

    currentState = nextState
    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/SuperstructureLoopTimeMS",
        (Clock.epochTime - startTime).inMilliseconds)
  }

  fun requestIdleCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.Idle() }
    returnCommand.name = "RequestIdleCommand"
    return returnCommand
  }

  fun requestForceHomeCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.ForceHome() }
    returnCommand.name = "RequestForceHomeCommand"
    return returnCommand
  }

  fun requestUnjamCommand(): Command {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.Unjam() }
    returnCommand.name = "RequestUnjamCommand"
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

  //  fun requestPrepClimbCommand(): Command {
  //    val returnCommand = runOnce { currentRequest = SuperstructureRequest.ExtendClimb() }
  //    returnCommand.name = "RequestPrepClimbCommand"
  //    return returnCommand
  //  }
  //
  //  fun requestClimbCommand(): Command {
  //    val returnCommand = runOnce { currentRequest = SuperstructureRequest.RetractClimb() }
  //    returnCommand.name = "RequestClimbCommand"
  //    return returnCommand
  //  }

  fun requestForceIntakeCommand(wantedAngle: Angle): Command {
    val returnCommand = runOnce {
      intake.currentRequest = Request.IntakeRequest.TargetingPosition(wantedAngle)
    }

    returnCommand.name = "RequestForceIntakeCommand"
    return returnCommand
  }

  fun requestForceClimbCommand(wantedPosition: Length): Command {
    val returnCommand = runOnce {
      climb.currentRequest = Request.ClimbRequest.TargetingPosition(wantedPosition)
    }

    returnCommand.name = "RequestForceClimbCommand"
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
      UNJAM,
      FORCE_HOME,
      PREP_SCORE,
      SCORE,
      SCORE_AND_INTAKE,
      INTAKE,
      PREP_CLIMB,
      CLIMB,
      EJECT
    }
  }
}
