package com.team4099.lib.hal

import edu.wpi.first.wpilibj.RobotController
import kotlin.time.Clock.System.now
import
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro
import org.team4099.lib.units.milli

object Clock {
  val fpgaTime: Time
    get() = RobotController.getFPGATime().micro.seconds

  val timestamp: Time
    get() = Logger.getTimestamp().micro.seconds

  val systemTime: Time
    get() = now().toEpochMilliseconds().milli.seconds
}
