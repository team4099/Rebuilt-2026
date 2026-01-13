package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.kilo
import org.team4099.lib.units.milli

object Constants {
  object Universal {
    val SIM_MODE = Tuning.SimType.SIM
    const val CANIVORE_NAME = "FalconVore"
    val LOG_FOLDER = "/media/sda1"

    val LOOP_PERIOD_TIME = 20.milli.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1

    const val SIMULATE_VISION = false

    enum class GamePiece {
      NONE
    }

    val ROBOT_WEIGHT = 135.pounds
    val ROBOT_MOI = 6.76.kilo.grams.meterSquared
  }

  object Tuning {
    const val TUNING_MODE = false
    const val DEBUGING_MODE = false

    enum class SimType {
      SIM,
      REPLAY
    }
  }

  object Joysticks {
    const val DRIVER_PORT = 0
    const val SHOTGUN_PORT = 1
    const val TECHNICIAN_PORT = 2

    const val THROTTLE_DEADBAND = 0.05
    const val TURN_DEADBAND = 0.05
  }

  object Alert {
    val TABS = arrayOf("Pre-match", "In-match")
  }
  object FieldConstants {
    val fieldLength = customFieldLayout.fieldLength.meters
    val fieldWidth = customFieldLayout.fieldWidth.meters

    val EMPTY_MAPLESIM_FIELD =
      object : SimulatedArena(object : FieldMap() {}) {
        override fun placeGamePiecesOnField() {}
      }
  }
}