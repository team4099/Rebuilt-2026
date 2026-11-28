package com.team4099.robot2026

import kotlin.time.ExperimentalTime
import org.wpilib.framework.RobotBase

class Main {
  companion object {
    @ExperimentalTime
    @JvmStatic
    fun main(args: Array<String>) {
      RobotBase.startRobot { Robot }
    }
  }
}
