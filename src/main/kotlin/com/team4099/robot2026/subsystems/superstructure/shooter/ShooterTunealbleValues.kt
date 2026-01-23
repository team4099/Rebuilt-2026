package com.team4099.robot2026.subsystems.superstructure.shooter

import com.team4099.lib.logging.LoggedTunableValue
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

object ShooterTunealbleValues {
  val idleVelocity = LoggedTunableValue(
    "Shooter/idleVoltage",
    0.degrees.perSecond,
    Pair({ it.inDegreesPerSecond }, { it.degrees.perSecond })
  )
}