package com.team4099.robot2026.config.constants

import org.team4099.lib.units.derived.degrees

object GyroConstants {
  val mountPitch =
      when (Constants.Universal.whoami) {
        else -> 180.0.degrees
      }
  val mountRoll =
      when (Constants.Universal.whoami) {
        else -> 0.0.degrees
      }
  val mountYaw =
      when (Constants.Universal.whoami) {
        else -> -90.0.degrees
      }
}
