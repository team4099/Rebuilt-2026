package com.team4099.robot2026.subsystems.superstructure.Intake.Rollers

import com.team4099.robot2025.subsystems.superstructure.Intake.Rollers.IntakeRollersIO
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts

class IntakeRollers(private val io: IntakeRollersIO) : ControlledByStateMachine() {

  val inputs = IntakeRollersIO.RollerInputs()

  var currentState = RollerStates.UNINITIALIZED

  var targetVoltage: ElectricalPotential = 0.volts
    private set

  var currentRequest: Request.RollersRequest = Request.RollersRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.RollersRequest.OpenLoop -> {
          targetVoltage = value.voltage
        }
      }
      field = value
    }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Rollers", inputs)
    CustomLogger.recordOutput("Rollers/currentState", currentState.toString())

    var nextState = currentState
    when (currentState) {
      RollerStates.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      RollerStates.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class RollerStates {
      OPEN_LOOP,
      UNINITIALIZED
    }

    fun fromRequestToState(request: Request.RollersRequest): RollerStates {
      return when (request) {
        is Request.RollersRequest.OpenLoop -> RollerStates.OPEN_LOOP
      }
    }
  }
}
