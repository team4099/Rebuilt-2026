package com.team4099.robot2026.subsystems.superstructure.feeder

import com.ctre.phoenix6.SignalLogger
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2026.config.constants.FeederConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.subsystems.superstructure.Request.FeederRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import java.util.function.Consumer
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.perSecond
import org.wpilib.command2.Command
import org.wpilib.command2.SubsystemBase
import org.wpilib.command2.sysid.SysIdRoutine
import org.wpilib.command2.sysid.SysIdRoutine.Mechanism
import org.wpilib.sysid.SysIdRoutineLog
import org.wpilib.units.Units.Volts
import org.wpilib.units.measure.Voltage as WPILibVoltage

class Feeder(private val io: FeederIO) : ControlledByStateMachine() {
  val inputs = FeederIO.FeederIOInputs()
  var currentState: FeederState = FeederState.UNINITIALIZED
  var currentRequest: FeederRequest = FeederRequest.Idle()
    set(value) {
      when (value) {
        is FeederRequest.OpenLoop -> feederVoltageTarget = value.voltage
        is FeederRequest.TargetVelocity -> feederVelocityTarget = value.velocity
        else -> {}
      }

      field = value
    }

  var feederVoltageTarget = 0.0.volts
    private set

  var feederVelocityTarget = 0.0.rotations.perSecond
    private set

  val isAtTargetedVelocity: Boolean
    get() =
        return (inputs.feederVelocity - feederVelocityTarget) <= FeederConstants.VELOCITY_TOLERANCE

  private val m_sysIdRoutine =
      SysIdRoutine(
          SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              Consumer { state: SysIdRoutineLog.State? ->
                run {
                  SignalLogger.writeString("state", state.toString())
                  CustomLogger.recordOutput("Feeder/sysIdState", state.toString())
                }
              }),
          Mechanism(
              { volts: WPILibVoltage ->
                currentRequest = Request.FeederRequest.OpenLoop(volts.`in`(Volts).volts)
              },
              null,
              object : SubsystemBase("Feeder") {}))

  val feederTestVel =
      LoggedTunableValue(
          "Feeder/testSpeedRotPerSec",
          FeederConstants.SCORE_VELOCITY,
          Pair({ it.inRotationsPerSecond }, { it.rotations.perSecond }))

  fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.quasistatic(direction)
  }

  fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.dynamic(direction)
  }

  override fun onLoop() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Feeder", inputs)

    CustomLogger.recordOutput("Feeder/CurrentState", currentState.name)
    CustomLogger.recordOutput("Feeder/CurrentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Feeder/isAtTargetedVelocity", isAtTargetedVelocity)

    var nextState = currentState
    when (currentState) {
      FeederState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.IDLE -> {
        io.setVoltage(FeederConstants.IDLE_VOLTAGE)
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.OPEN_LOOP -> {
        io.setVoltage(feederVoltageTarget)
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.TARGETING_VELOCITY -> {
        io.setVelocity(feederVelocityTarget)
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
      TARGETING_VELOCITY
    }

    inline fun fromRequestToState(request: FeederRequest): FeederState {
      return when (request) {
        is FeederRequest.OpenLoop -> FeederState.OPEN_LOOP
        is FeederRequest.Idle -> FeederState.IDLE
        is FeederRequest.TargetVelocity -> FeederState.TARGETING_VELOCITY
      }
    }
  }
}
