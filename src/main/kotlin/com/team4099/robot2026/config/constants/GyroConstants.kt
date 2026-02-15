package com.team4099.robot2026.config.constants

import org.team4099.lib.units.derived.degrees

object GyroConstants {
  val mountPitch
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.COMPBOT,
          Constants.WHOAMI.ALPHABOT -> 2.989051342010498.degrees
          Constants.WHOAMI.ALPHABOT -> 0.degrees
          Constants.WHOAMI.TESTBOT -> 180.0.degrees
          else -> 0.degrees
        }

  val mountRoll
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.COMPBOT,
          Constants.WHOAMI.ALPHABOT -> -179.7517547607422.degrees
          Constants.WHOAMI.ALPHABOT -> 0.degrees
          Constants.WHOAMI.TESTBOT -> 0.0.degrees
          else -> 0.degrees
        }

  val mountYaw
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.COMPBOT,
          Constants.WHOAMI.ALPHABOT -> -89.76551055908203.degrees
          Constants.WHOAMI.ALPHABOT -> 180.degrees
          Constants.WHOAMI.TESTBOT -> -90.0.degrees
          else -> 0.degrees
        }
}
