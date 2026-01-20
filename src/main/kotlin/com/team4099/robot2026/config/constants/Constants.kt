package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.amps
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
    const val DISABLE_COLLISIONS = true

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
  object Hopper {
    const val HOPPER_MOTOR_ID = -1337
    const val HOPPER_MOTOR_NAME = "Hopepr Motor"
    const val HOPPER_MOTOR_SPEED = -1337
    const val GEAR_RATIO: Double = 1.0 / 1.0
    const val MAX_ACCELERATION: Double = -1337.0
    const val MAX_VELOCITY: Double = -1337.0
    val VOLTAGE_COMPENSATION = -1337.amps
  const val ENCODER_TO_MECHANISM_GEAR_RATIO = -1337
  }
}
