package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo

object HopperConstants {
  const val GEAR_RATIO: Double = 1.0 / 27.0

  val STATOR_CURRENT_LIMIT = 40.amps
  val SUPPLY_CURRENT_LIMIT = 40.amps

  val VOLTAGE_COMPENSATION = 12.volts

  val IDLE_VOLTAGE = 0.0.volts
  val MOMENT_OF_INERTIA = 0.0008656281.kilo.grams.meterSquared
}
