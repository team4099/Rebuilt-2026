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

    const val SIMULATE_VISION = true
    const val DISABLE_COLLISIONS = false

    val ROBOT_WEIGHT = 125.pounds
    val ROBOT_MOI = 6.3.kilo.grams.meterSquared

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

  object Intake {
    const val INTAKE_PIVOT_MOTOR_ID = 41
    const val INTAKE_ROLLERS_MOTOR_ID = 42
  }

  object Hopper {
    const val HOPPER_MOTOR_ID = 46
  }

  object Shooter {
    const val LEADER_MOTOR_ID = 51
    const val FOLLOWER_MOTOR_ID = 52
  }

  object Feeder {
    const val FEEDER_MOTOR_ID = 56
  }

  object Climb {
    const val CLIMB_MOTOR_ID = 61
  }

  object ClusterScore {
    val COMPACTNESS_WEIGHT = 1.0
    val DISTANCE_WEIGHT = 1.0
    val MIN_SCORE_CLUSTER_SIZE = 1.0
  }
}
