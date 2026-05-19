package com.team4099.robot2026.subsystems.superstructure.hopper

import com.ctre.phoenix6.SignalLogger
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2026.config.constants.HopperConstants
import com.team4099.robot2026.subsystems.superstructure.Request.HopperRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

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

  private val m_sysIdRoutine =
      SysIdRoutine(
          SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(3.0), // Reduce dynamic step voltage to 4 to prevent brownout
              null // Use default timeout (10 s)
              )
          // Log state with Phoenix SignalLogger class
          { state: SysIdRoutineLog.State? ->
                run {
                  SignalLogger.writeString("state", state.toString())
                  CustomLogger.recordOutput("Hopper/sysIdState", state.toString())
                }
              },
          Mechanism(
              { volts: WPILibVoltage ->
                currentRequest = HopperRequest.OpenLoop(volts.`in`(Volts).volts)
              },
              null,
              object : SubsystemBase("Hopper") {}))

  val hopperTestVel =
      LoggedTunableValue(
          "Hopper/testSpeedRotPerSec",
          HopperConstants.VELOCITIES.SCORE_VELOCITY,
          Pair({ it.inRotationsPerSecond }, { it.rotations.perSecond }))

  init {
    if (RobotBase.isReal()) {
      io.configurePIDCurrent(
          HopperConstants.PID.REAL_KP, HopperConstants.PID.REAL_KI, HopperConstants.PID.REAL_KD)
      io.configureFFCurrent(
          HopperConstants.PID.REAL_KS, HopperConstants.PID.REAL_KV, HopperConstants.PID.REAL_KA)
    } else {
      io.configurePIDVoltage(
          HopperConstants.PID.SIM_KP, HopperConstants.PID.SIM_KI, HopperConstants.PID.SIM_KD)
      io.configureFFVoltage(
          HopperConstants.PID.SIM_KS, HopperConstants.PID.SIM_KV, HopperConstants.PID.SIM_KA)
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

  fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.quasistatic(direction)
  }

  fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.dynamic(direction)
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
