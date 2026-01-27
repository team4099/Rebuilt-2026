package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ClimbConstants {
  val DOWNWARDS_EXTENSION_LIMIT = 0.0.inches
  val UPWARDS_EXTENSION_LIMIT = 8.inches // TODO: might need to tune height
  val CLIMB_MASS = 5.5444326.pounds

  val CLIMB_TOLERANCE = 0.5.inches

  val GEAR_RATIO = 30 / 1.0 // TODO: change as needed depending on our use case/need for torque
  val DRUM_DIAMETER = 1.inches
  val VOLTAGE_COMPENSATION = 12.volts

  val SUPPLY_CURRENT_LIMIT = 40.amps
  val STATOR_CURRENT_LIMIT = 40.amps

  val MAX_VELOCITY = 50.inches.perSecond
  val MAX_ACCELERATION = 50.inches.perSecond.perSecond

  object PID {
    val REAL_KP = 0.0.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.0.volts / (1.inches.perSecond)

    val REAL_KS = 0.0.volts
    val REAL_KG = 0.0.volts

    val SIM_KP = 0.0.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.0.volts / (1.inches.perSecond)

    val SIM_KS = 0.0.volts
    val SIM_KG = 0.0.volts
  }
}
