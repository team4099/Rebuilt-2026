package com.team4099.robot2026.subsystems.climb

import com.team4099.robot2026.config.constants.ClimbConstants
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts
import com.team4099.robot2026.subsystems.superstructure.Request.ClimbRequest as ClimbRequest

class Climb(val io: ClimbIO) : SubsystemBase() {
    val inputs = ClimbIO.ClimbInputs()

    val upperLimitReached: Boolean
        get() = inputs.climbPosition >= ClimbConstants.UPWARDS_EXTENSION_LIMIT

    val lowerLimitReached: Boolean
        get() = inputs.climbPosition <= ClimbConstants.DOWNWARDS_EXTENSION_LIMIT

    var isHomed = false

    var currentState: ClimbState = ClimbState.UNINITIALIZED
    var currentRequest: ClimbRequest = ClimbRequest.OpenLoop(0.0.volts)
        set(value) {
            when (value) {
                is ClimbRequest.OpenLoop -> climbVoltageTarget = value.voltage
                is ClimbRequest.TargetingPosition -> climbPositionTarget = value.position
            }
            field = value
        }

    var climbPositionTarget = 0.0.inches
        private set
    var climbVoltageTarget = 0.0.volts
        private set

    val isAtTargetedPosition: Boolean
        get() =
            (
                currentRequest is ClimbRequest.TargetingPosition &&
                        (inputs.climbPosition - climbPositionTarget).absoluteValue <=
                        ClimbConstants.CLIMB_TOLERANCE
                )

    init {

        if (RobotBase.isReal()) {
            ClimbTunableValues.kP.initDefault(ClimbConstants.PID.REAL_KP)
            ClimbTunableValues.kI.initDefault(ClimbConstants.PID.REAL_KI)
            ClimbTunableValues.kD.initDefault(ClimbConstants.PID.REAL_KD)
        } else {
            ClimbTunableValues.kP.initDefault(ClimbConstants.PID.SIM_KP)
            ClimbTunableValues.kI.initDefault(ClimbConstants.PID.SIM_KI)
            ClimbTunableValues.kD.initDefault(ClimbConstants.PID.SIM_KD)
        }

        io.configPID(
            ClimbTunableValues.kP.get(),
            ClimbTunableValues.kI.get(),
            ClimbTunableValues.kD.get()
        )

        // Removed FF because a telescoping climber shouldn't need FF and can suffice with pure PID
    }

    override fun periodic() {
        io.updateInputs(inputs)

        if (ClimbTunableValues.kP.hasChanged() ||
            ClimbTunableValues.kI.hasChanged() ||
            ClimbTunableValues.kD.hasChanged()
        ) {
            io.configPID(
                ClimbTunableValues.kP.get(),
                ClimbTunableValues.kI.get(),
                ClimbTunableValues.kD.get()
            )
        }

        CustomLogger.processInputs("Climb", inputs)

        CustomLogger.recordOutput("Climb/currentState", currentState.name)
        CustomLogger.recordOutput("Climb/currentRequest", currentRequest.javaClass.simpleName)

        CustomLogger.recordOutput("Climb/isAtTargetPosition", isAtTargetedPosition)
        CustomLogger.recordOutput("Climb/climbPositionTarget", climbPositionTarget.inInches)
        CustomLogger.recordDebugOutput("Climb/upperLimitReached", upperLimitReached)
        CustomLogger.recordDebugOutput("Climb/lowerLimitReached", lowerLimitReached)

        var nextState = currentState

        when (currentState) {
            ClimbState.UNINITIALIZED -> {
                nextState = fromClimbRequestToState(currentRequest)
                io.zeroEncoder()
            }
            ClimbState.OPEN_LOOP -> {
                setVoltage(climbVoltageTarget)
                nextState = fromClimbRequestToState(currentRequest)
            }
            ClimbState.TARGETING_POSITION -> {
                io.setPosition(climbPositionTarget)
                nextState = fromClimbRequestToState(currentRequest)
            }
        }
        currentState = nextState
    }

    fun setVoltage(targetVoltage: ElectricalPotential) {
        if (
            (upperLimitReached && targetVoltage > 0.0.volts) ||
                (lowerLimitReached && targetVoltage < 0.0.volts)
        ) {
            io.setVoltage(0.0.volts)
        } else {
            io.setVoltage(targetVoltage)
        }
    }

    companion object {
        enum class ClimbState {
            UNINITIALIZED,
            OPEN_LOOP,
            TARGETING_POSITION;

            inline fun equivalentToRequest(request: ClimbRequest): Boolean {
                return (request is ClimbRequest.OpenLoop && this == OPEN_LOOP) || 
                        (request is ClimbRequest.TargetingPosition && this == TARGETING_POSITION)
            }
        }

        inline fun fromClimbRequestToState(request: ClimbRequest): ClimbState {
            return when (request) {
                is ClimbRequest.OpenLoop -> ClimbState.OPEN_LOOP
                is ClimbRequest.TargetingPosition -> ClimbState.TARGETING_POSITION
            }
        }
    }
}