package com.team4099.robot2026.subsystems.superstructure.feeder

import com.team4099.robot2025.subsystems.Feeders.FeederIO
import com.team4099.robot2026.subsystems.superstructure.Request.FeederRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.volts

class Feeder(private val io: FeederIO) : ControlledByStateMachine() {
  val inputs = FeederIO.FeederIOInputs()
  var currentState: FeederState = FeederState.UNINITIALIZED
  var currentRequest: FeederRequest = FeederRequest.Idle()
    set(value) {
      when (value) {
        is FeederRequest.OpenLoop -> feederVoltageTarget = value.voltage
        else -> {}
      }

      field = value
    }

  var feederVoltageTarget = 0.0.volts
    private set

  override fun onLoop() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Feeder", inputs)

    CustomLogger.recordOutput("Feeder/CurrentState", currentState.name)
    CustomLogger.recordOutput("Feeder/CurrentRequest", currentRequest.javaClass.simpleName)

    var nextState = currentState
    when (currentState) {
      FeederState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.IDLE -> {
        io.setVoltage(0.0.volts)
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.OPEN_LOOP -> {
        io.setVoltage(feederVoltageTarget)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class FeederState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP,
    }

    inline fun fromRequestToState(request: FeederRequest): FeederState {
      return when (request) {
        is FeederRequest.OpenLoop -> FeederState.OPEN_LOOP
        is FeederRequest.Idle -> FeederState.IDLE
      }
    }
  }
}
