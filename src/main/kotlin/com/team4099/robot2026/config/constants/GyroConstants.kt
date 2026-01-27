package com.team4099.robot2026.config.constants

import org.team4099.lib.units.derived.degrees

object GyroConstants {
  val mountPitch
    get() = when (Constants.Universal.whoami) {
        else -> 180.0.degrees
      }
  val mountRoll
    get() = when (Constants.Universal.whoami) {
        else -> 0.0.degrees
      }
  val mountYaw
    get() = when (Constants.Universal.whoami) {
        else -> -90.0.degrees
      }
}
