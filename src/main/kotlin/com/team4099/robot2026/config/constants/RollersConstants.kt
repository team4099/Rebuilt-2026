package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts

object RollersConstants {
  val GEAR_RATIO = 1.0 / 1.0
  val VOLTAGE_COMPENSATION = 12.volts

  val IDLE_VOLTAGE = 0.0.volts
  val EJECT_VOLTAGE = (-3.0).volts
  val INTAKE_VOLTAGE = 12.0.volts
  val SCORE_ASSISTING_VOLTAGE = 0.volts

  val MOMENT_OF_INERTIA = 1.0.grams.meterSquared // TODO: Change

  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 100.0.amps

  val FUEL_STALL_CURRENT = 35.amps
  val FUEL_STALL_TIME_THRESHOLD = 1.seconds
}
