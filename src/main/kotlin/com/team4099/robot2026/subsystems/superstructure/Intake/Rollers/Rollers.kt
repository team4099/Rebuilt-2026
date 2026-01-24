package com.team4099.robot2026.subsystems.superstructure.Intake.Rollers

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts

class Rollers(val io: RollersIO) : ControlledByStateMachine() {

  val inputs = RollersIO.RollerInputs()

  var currentState = rollerStates.UNINITIALIZED

  var lastRollerRunTime = Clock.fpgaTime

  val hasCoral: Boolean
    get() {
      return inputs.rollerStatorCurrent > RollersConstants.CORAL_CURRENT_THRESHOLD &&
          (Clock.fpgaTime - lastRollerRunTime >= RollersConstants.CORAL_DETECTION_THRESHOLD)
    }

  val hasAlgae: Boolean
    get() {
      return inputs.rollerStatorCurrent > RollersConstants.ALGAE_CURRENT_THRESHOLD &&
          (Clock.fpgaTime - lastRollerRunTime >= RollersConstants.ALGAE_DETECTION_THRESHOLD)
    }

  var targetVoltage: ElectricalPotential = 0.0.volts

  var lastTransitionTime = 0.seconds
  var lastAlgaeTriggerTime = 0.seconds

  var currentRequest: Request.RollersRequest = Request.RollersRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.RollersRequest.OpenLoop -> {
          targetVoltage = value.voltage
        }
        else -> {}
      }
      field = value
    }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Rollers", inputs)
    CustomLogger.recordOutput("Rollers/currentState", currentState.toString())

    var nextState = currentState
    when (currentState) {
      rollerStates.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      rollerStates.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)

        if (lastAlgaeTriggerTime < lastTransitionTime && hasAlgae)
            lastAlgaeTriggerTime = Clock.fpgaTime

        nextState = fromRequestToState(currentRequest)
      }
    }
    if (nextState != currentState) lastTransitionTime = Clock.fpgaTime
    currentState = nextState
  }

  companion object {
    enum class rollerStates {
      OPEN_LOOP,
      UNINITIALIZED
    }

    fun fromRequestToState(request: Request.RollersRequest): rollerStates {
      return when (request) {
        is Request.RollersRequest.OpenLoop -> rollerStates.OPEN_LOOP
      }
    }
  }
}
