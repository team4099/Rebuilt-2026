package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object ClimbConstants {
  val DOWNWARDS_EXTENSION_LIMIT = 0.0.inches
  val UPWARDS_EXTENSION_LIMIT = 40.inches // not exact, change later after mounting the climber on
  val CLIMB_MASS = 2.5.kilo.grams // TODO: adjust depending on CAD

  val CLIMB_TOLERANCE = 0.5.inches

  val GEAR_RATIO = 30 / 1.0 // TODO: change as needed depending on our use case/need for torque
  val DRUM_DIAMETER =
      1.inches // TODO: figure out diameter of the drum that we wind the spool around for climb,
  // or the code won't work
  val VOLTAGE_COMPENSATION = 12.volts

  val SUPPLY_CURRENT_LIMIT = 40.amps
  val STATOR_CURRENT_LIMIT = 40.amps

  val MAX_VELOCITY = 50.inches.perSecond
  val MAX_ACCELERATION = 50.inches.perSecond.perSecond

  object PID {
    val REAL_KP = 0.0.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.0.volts / (1.inches.perSecond)

    val SIM_KP = 0.0.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.0.volts / (1.inches.perSecond)
  }
}
