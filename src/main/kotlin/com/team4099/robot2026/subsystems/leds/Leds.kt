package com.team4099.robot2026.subsystems.leds

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.config.constants.LedConstants.CandleState
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier
import org.team4099.lib.units.base.inMilliseconds

class Leds(
    var isAligningSupplier: Supplier<Boolean>,
    var stateSupplier: Supplier<Superstructure.Companion.SuperstructureStates>,
    vararg candles: LedIO,
) : SubsystemBase() {
  var io = candles.toList()
  var inputs = List(io.size) { LedIO.LEDIOInputs() }

  var state = CandleState.NOTHING
  private var lastState: CandleState? = null

  override fun periodic() {
    val startTime = Clock.timestamp

    state =
        if (DriverStation.isDisabled()) {
          if (!DriverStation.getAlliance().isPresent) CandleState.NOTHING
          else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
              CandleState.BLUE_DISABLED
          else CandleState.RED_DISABLED
        } else if (isAligningSupplier.get()) CandleState.IS_ALIGNING
        else if (stateSupplier.get() == Superstructure.Companion.SuperstructureStates.INTAKE)
            CandleState.INTAKING_FUEL
        else if (stateSupplier.get() == Superstructure.Companion.SuperstructureStates.SCORE)
            CandleState.SHOOTING
        else CandleState.NOTHING

    CustomLogger.recordOutput("Led/state", state.name)

    for (instance in io.indices) {
      if (lastState != state) io[instance].turnOff()
      io[instance].setState(state)
      lastState = state

      CustomLogger.processInputs("Led/$instance", inputs[instance])
    }

    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/LEDLoopTimeMS", (Clock.timestamp - startTime).inMilliseconds)
  }
}
