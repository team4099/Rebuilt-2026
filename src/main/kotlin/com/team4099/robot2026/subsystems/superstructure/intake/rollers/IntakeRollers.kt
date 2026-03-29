package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.team4099.robot2026.config.constants.RollersConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.ironmaple.simulation.IntakeSimulation
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

  val intakeSimulation: IntakeSimulation?
    get() = io.intakeSimulation

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("rollers", inputs)
    CustomLogger.recordOutput("rollers/currentState", currentState.toString())

    var nextState = currentState
    when (currentState) {
      RollerStates.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      RollerStates.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)

        if (targetVoltage == RollersConstants.INTAKE_VOLTAGE) io.intakeSimulation?.startIntake()
        else io.intakeSimulation?.stopIntake()

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
