package com.team4099.robot2026.subsystems.climb

import com.team4099.lib.logging.LoggedTunableValue
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ClimbTunableValues {
  val kP = LoggedTunableValue("Climb/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  val kI =
      LoggedTunableValue(
          "Climb/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds }))
  val kD =
      LoggedTunableValue(
          "Climb/kI", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond }))
  val kS =
    LoggedTunableValue(
      "Climb/kS", Pair({ it.inVolts }, { it.volts }))
  val kG =
    LoggedTunableValue(
      "Climb/kG", Pair({ it.inVolts }, { it.volts }))
}
