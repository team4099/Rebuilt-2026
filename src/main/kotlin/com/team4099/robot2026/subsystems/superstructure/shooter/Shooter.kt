package com.team4099.robot2026.subsystems.superstructure.shooter

import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

class Shooter(private val io: ShooterIO) : ControlledByStateMachine() {
  val inputs = ShooterIO.ShooterInputs()
  var shooterVoltageTarget: ElectricalPotential = 0.0.volts
    private set

  var shooterVelocityTarget: AngularVelocity = 0.0.degrees.perSecond
    private set

  val isAtTargetedVelocity: Boolean
    get() =
        (currentRequest is Request.ShooterRequest.TargetVelocity &&
            (inputs.shooterLeaderVelocity - shooterVelocityTarget).absoluteValue <
                ShooterConstants.SHOOTER_TOLERANCE)

  var currentState: ShooterState = ShooterState.UNINITIALIZED
  var currentRequest: Request.ShooterRequest = Request.ShooterRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.ShooterRequest.OpenLoop -> {
          shooterVoltageTarget = value.shooterVoltage
        }
        is Request.ShooterRequest.TargetVelocity -> {
          shooterVelocityTarget = value.targetVelocity
        }
        else -> {}
      }
      field = value
    }

  init {
    if (RobotBase.isReal()) {
      io.configurePID(
          ShooterConstants.PID.REAL_KP, ShooterConstants.PID.REAL_KI, ShooterConstants.PID.REAL_KD)
      io.configureFF(
          ShooterConstants.PID.REAL_KS, ShooterConstants.PID.REAL_KV, ShooterConstants.PID.REAL_KA)
    } else {
      io.configurePID(
          ShooterConstants.PID.SIM_KP, ShooterConstants.PID.SIM_KI, ShooterConstants.PID.SIM_KD)
      io.configureFF(
          ShooterConstants.PID.SIM_KS, ShooterConstants.PID.SIM_KV, ShooterConstants.PID.SIM_KA)
    }
  }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Shooter", inputs)
    CustomLogger.recordOutput(
        "Shooter/targetAngularVelocity", shooterVelocityTarget.inDegreesPerSecond)
    CustomLogger.recordOutput("Shooter/targetVoltage", shooterVoltageTarget.inVolts)
    CustomLogger.recordOutput("Shooter/currentState", currentState)
    CustomLogger.recordOutput("Shooter/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Shooter/isAtTargetedVelocity", isAtTargetedVelocity)

    var nextState = currentState

    when (currentState) {
      ShooterState.UNINITIALIZED -> {
        nextState = fromShooterRequestToState(currentRequest)
      }
      ShooterState.OPEN_LOOP -> {
        io.setVoltage(shooterVoltageTarget)
        nextState = fromShooterRequestToState(currentRequest)
      }
      ShooterState.TARGET_VELOCITY -> {
        io.setVelocity(shooterVelocityTarget)
        nextState = fromShooterRequestToState(currentRequest)
      }
      ShooterState.IDLE -> {
        io.setVoltage(ShooterConstants.IDLE_VOLTAGE)
        nextState = fromShooterRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class ShooterState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP,
      TARGET_VELOCITY
    }

    inline fun fromShooterRequestToState(request: Request.ShooterRequest): ShooterState {
      return when (request) {
        is Request.ShooterRequest.TargetVelocity -> ShooterState.TARGET_VELOCITY
        is Request.ShooterRequest.OpenLoop -> ShooterState.OPEN_LOOP
        is Request.ShooterRequest.Idle -> ShooterState.IDLE
      }
    }
  }
}
