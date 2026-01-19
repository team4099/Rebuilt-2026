package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ShooterConstants {
  // TODO every single constant value is random
  val GEAR_RATIO: Double = 1.0 / 1.0
  val SUPPLY_CURRENT_LIMIT = 0.1.amps
  val STATOR_CURRENT_LIMIT = 0.2.amps
  val VOLTAGE_COMPENSATION = 12.0.volts

  val MAX_VELOCITY = 0.1.degrees.perSecond
  val MAX_ACCELERATION = 0.1.degrees.perSecond.perSecond
}
