package com.team4099.robot2026.subsystems.superstructure.hopper

import com.team4099.robot2026.config.constants.HopperConstants
import com.team4099.robot2026.subsystems.superstructure.Request.HopperRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perMinute

class Hopper(private val io: HopperIO) : ControlledByStateMachine() {
  var hopperVoltageTarget = 0.0.volts
    private set

  var hopperVelocityTarget = 0.rotations.perMinute

  val inputs = HopperIO.HopperIOInputs()
  var currentState: HopperState = HopperState.UNINITIALIZED
  var currentRequest: HopperRequest = HopperRequest.Idle()
    set(value) {
      when (value) {
        is HopperRequest.OpenLoop -> hopperVoltageTarget = value.voltage
        is HopperRequest.TargetVelocity -> hopperVelocityTarget = value.velocity
        else -> {}
      }
      field = value
    }

  init {
    if (RobotBase.isReal()) {
      io.configurePIDCurrent(
        HopperConstants.PID.REAL_KP, HopperConstants.PID.REAL_KI, HopperConstants.PID.REAL_KD
      )
      io.configureFFCurrent(
        HopperConstants.PID.REAL_KS, HopperConstants.PID.REAL_KV, HopperConstants.PID.REAL_KA
      )
    } else {
      io.configurePIDVoltage(
        HopperConstants.PID.SIM_KP, HopperConstants.PID.SIM_KI,HopperConstants.PID.SIM_KD
      )
      io.configureFFVoltage(
        HopperConstants.PID.SIM_KS, HopperConstants.PID.SIM_KV, HopperConstants.PID.SIM_KA
      )
    }
  }

  override fun onLoop() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Hopper", inputs)

    CustomLogger.recordOutput("Hopper/CurrentState", currentState.name)
    CustomLogger.recordOutput("Hopper/CurrentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Hopper/targetVoltage", hopperVoltageTarget.inVolts)
    CustomLogger.recordOutput("Hopper/targetVelocityDPS", hopperVelocityTarget.inDegreesPerSecond)

    var nextState = currentState
    when (currentState) {
      HopperState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      HopperState.IDLE -> {
        io.setVelocity(HopperConstants.VELOCITIES.IDLE_VELOCITY)
        nextState = fromRequestToState(currentRequest)
      }
      HopperState.OPEN_LOOP -> {
        io.setVoltage(hopperVoltageTarget)
        nextState = fromRequestToState(currentRequest)
      }
      HopperState.TARGETING_VELOCITY -> {
        io.setVelocity(hopperVelocityTarget)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class HopperState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP,
      TARGETING_VELOCITY
    }

    inline fun fromRequestToState(request: HopperRequest): HopperState {
      return when (request) {
        is HopperRequest.OpenLoop -> HopperState.OPEN_LOOP
        is HopperRequest.Idle -> HopperState.IDLE
        is HopperRequest.TargetVelocity -> HopperState.TARGETING_VELOCITY
      }
    }
  }
}
