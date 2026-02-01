package com.team4099.robot2026.subsystems.superstructure.intake

import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Intake(private val io: IntakeIO) : ControlledByStateMachine() {
  val inputs = IntakeIO.IntakeIOInputs()

  var pivotPositionTarget: Angle = 0.0.degrees
    private set

  var pivotVoltageTarget: ElectricalPotential = 0.0.volts
    private set

  var currentState: IntakeState = IntakeState.UNINITIALIZED

  var currentRequest: Request.IntakeRequest = Request.IntakeRequest.ZeroPivot()
    set(value) {
      when (value) {
        is Request.IntakeRequest.OpenLoop -> {
          pivotVoltageTarget = value.pivotVoltage
        }
        is Request.IntakeRequest.TargetingPosition -> {
          pivotPositionTarget = value.pivotPosition
        }
        else -> {}
      }
      field = value
    }

  val isAtTargetedPosition: Boolean
    get() =
        currentRequest is Request.IntakeRequest.TargetingPosition &&
            (inputs.position - pivotPositionTarget).absoluteValue <=
                IntakeConstants.INTAKE_TOLERANCE

  init {
    if (RobotBase.isReal()) {
      io.configPID(IntakeConstants.PID.REAL_PIVOT_KP, IntakeConstants.PID.REAL_PIVOT_KI, IntakeConstants.PID.REAL_PIVOT_KD)
      io.configFF(IntakeConstants.PID.PIVOT_KG, IntakeConstants.PID.PIVOT_KS, IntakeConstants.PID.PIVOT_KV, IntakeConstants.PID.PIVOT_KA)
    } else {
      io.configPID(IntakeConstants.PID.SIM_PIVOT_KP, IntakeConstants.PID.SIM_PIVOT_KI, IntakeConstants.PID.SIM_PIVOT_KD)
      io.configFF(IntakeConstants.PID.SIM_PIVOT_KG, IntakeConstants.PID.SIM_PIVOT_KS, IntakeConstants.PID.SIM_PIVOT_KV, IntakeConstants.PID.SIM_PIVOT_KA)
    }
  }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("intake", inputs)

    CustomLogger.recordOutput("intake/currentState", currentState.toString())
    CustomLogger.recordOutput("intake/currentRequest", currentRequest.javaClass.simpleName)

    var nextState: IntakeState
    CustomLogger.recordOutput("intake/pivotTargetPosition", pivotPositionTarget.inDegrees)
    CustomLogger.recordOutput("intake/pivotTargetVoltage", pivotVoltageTarget.inVolts)

    CustomLogger.recordOutput("intake/isAtTargetedPosition", isAtTargetedPosition)

    when (currentState) {
      IntakeState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      IntakeState.ZEROING_PIVOT -> {
        io.zeroPivot()
        currentRequest = Request.IntakeRequest.OpenLoop(0.volts)
        nextState = fromRequestToState(currentRequest)
      }
      IntakeState.OPEN_LOOP -> {
        io.setVoltage(pivotVoltageTarget)
        nextState = fromRequestToState(currentRequest)
      }
      IntakeState.TARGETING_POSITION -> {
        io.setPosition(pivotPositionTarget)
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState
  }

  companion object {
    enum class IntakeState {
      UNINITIALIZED,
      ZEROING_PIVOT,
      TARGETING_POSITION,
      OPEN_LOOP
    }

    inline fun fromRequestToState(request: Request.IntakeRequest): IntakeState {
      return when (request) {
        is Request.IntakeRequest.OpenLoop -> IntakeState.OPEN_LOOP
        is Request.IntakeRequest.TargetingPosition -> IntakeState.TARGETING_POSITION
        is Request.IntakeRequest.ZeroPivot -> IntakeState.ZEROING_PIVOT
      }
    }
  }
}
