package com.team4099.robot2026.config.constants

import com.team4099.robot2026.util.ArgParser.argToBool
import com.team4099.robot2026.util.ArgParser.argToEnum
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.kilo
import org.team4099.lib.units.milli
import org.team4099.lib.units.perSecond

object Constants {
  object Universal {
    val gravity = 9.81.meters.perSecond.perSecond
    val SIM_MODE = Tuning.SimType.SIM
    const val REAL_FIELD = false

    const val CTRE_CONFIG_TIMEOUT = 0
    const val EPSILON = 1E-9

    val SLOW_STATUS_FRAME_TIME = 255.milli.seconds
    const val CANIVORE_NAME = "FalconVore"
    val LOG_FOLDER = "/home/lvuser/logs/"

    val LOOP_PERIOD_TIME = 20.milli.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1

    const val SIMULATE_VISION = false
    const val DISABLE_COLLISIONS = false

    val ROBOT_WEIGHT = 135.pounds
    val ROBOT_MOI = 6.76.kilo.grams.meterSquared

    val whoami = argToEnum<WHOAMI>(System.getProperty("robot"))
  }

  enum class WHOAMI {
    COMPBOT,
    ALPHABOT,
    TESTBOT
  }

  object Tuning {
    val TUNING_MODE = argToBool(System.getProperty("tuning"))
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

  object Shooter {
    const val LEADER_MOTOR_ID = 42
    const val FOLLOWER_MOTOR_ID = 43
  }
}
