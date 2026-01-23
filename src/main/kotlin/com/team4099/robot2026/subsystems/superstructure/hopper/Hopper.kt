package com.team4099.robot2026.subsystems.superstructure.hopper

import com.team4099.robot2026.config.constants.HopperConstants
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIO
import com.team4099.robot2026.subsystems.superstructure.Request.HopperRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.volts

class Hopper(private val io: HopperIO) : ControlledByStateMachine() {
  var hopperVoltageTarget = 0.0.volts
    private set

  val inputs = HopperIO.HopperIOInputs()
  var currentState: HopperState = HopperState.UNINITIALIZED
  var currentRequest: HopperRequest = HopperRequest.Idle()
    set(value) {
      when (value) {
        is HopperRequest.OpenLoop -> hopperVoltageTarget = value.voltage
        else -> {}
      }
      field = value
    }

  override fun onLoop() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Hopper", inputs)

    CustomLogger.recordOutput("Hopper/CurrentState", currentState.name)
    CustomLogger.recordOutput("Hopper/CurrentRequest", currentRequest.javaClass.simpleName)

    var nextState = currentState
    when (currentState) {
      HopperState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      HopperState.IDLE -> {
        io.setVoltage(HopperConstants.IDLE_VOLTAGE)
        nextState = fromRequestToState(currentRequest)
      }
      HopperState.OPEN_LOOP -> {
        io.setVoltage(hopperVoltageTarget)
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
    }

    inline fun fromRequestToState(request: HopperRequest): HopperState {
      return when (request) {
        is HopperRequest.OpenLoop -> HopperState.OPEN_LOOP
        is HopperRequest.Idle -> HopperState.IDLE
      }
    }
  }
}
