package com.team4099.lib.hal

import kotlin.time.Clock.System.now
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro
import org.team4099.lib.units.nano

object Clock {
  val timestamp: Time
    get() = Logger.getTimestamp().micro.seconds

  val epochTime: Time
    get() = now().epochSeconds.seconds + now().nanosecondsOfSecond.nano.seconds
}
