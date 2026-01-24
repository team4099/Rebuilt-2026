package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo

object RollersConstants {
  val GEAR_RATIO = 1.0 / 1.0
  val VOLTAGE_COMPENSATION = 12.volts

  val FUEL_CURRENT_THRESHOLD = 30.0.amps
  val FUEL_DETECTION_THRESHOLD = 2.0.seconds

  val IDLE_VOLTAGE = 0.0.volts
  val EJECT_VOLTAGE = (-3.0).volts
  val INTAKE_VOLTAGE = 6.0.volts

  val MOMENT_OF_INERTIA = 0.0.kilo.grams.meterSquared // TODO: Change

  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 40.0.amps
}
