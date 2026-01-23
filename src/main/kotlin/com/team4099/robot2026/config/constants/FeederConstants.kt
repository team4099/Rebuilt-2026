package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts

object FeederConstants {
  val STATOR_CURRENT_LIMIT = 40.0.amps
  val SUPPLY_CURRENT_LIMIT = 60.0.amps
  val GEAR_RATIO = 24.0 / 12.0
  val VOLTAGE_COMPENSATION = 12.0.volts
  // TODO: Change this value later
  val MOMENT_OF_INERTIA = 0.0.grams.meterSquared
}
