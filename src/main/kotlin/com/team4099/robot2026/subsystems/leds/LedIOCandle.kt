package com.team4099.robot2026.subsystems.leds

import com.ctre.phoenix6.configs.CANdleConfiguration
import com.ctre.phoenix6.controls.EmptyAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue
import com.team4099.robot2026.config.constants.LedConstants

class LedIOCandle(val id: Int) : LedIO {
  private val candle = CANdle(id)
  private val configs = CANdleConfiguration()

  init {
    candle.clearStickyFaults()

    configs.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs

    candle.configurator.apply(configs)
  }

  override fun setState(state: LedConstants.CandleState) {
    candle.setControl(state.request)
  }

  override fun turnOff() {
    candle.setControl(EmptyAnimation(0))
  }
}
