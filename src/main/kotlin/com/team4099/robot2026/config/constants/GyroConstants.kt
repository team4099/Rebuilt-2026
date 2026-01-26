package com.team4099.robot2026.config.constants

import com.team4099.robot2026.util.ArgParser
import org.team4099.lib.units.derived.degrees

object GyroConstants {
  val mountPitch =
    when (ArgParser.robotType) {
      else -> 180.0.degrees
    }
  val mountRoll =
    when (ArgParser.robotType) {
      else -> 0.0.degrees
    }
  val mountYaw =
    when (ArgParser.robotType) {
      else -> -90.0.degrees
    }
}
