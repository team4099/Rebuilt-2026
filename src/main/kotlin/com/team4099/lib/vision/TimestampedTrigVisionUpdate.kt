package com.team4099.lib.vision

import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time

data class TimestampedTrigVisionUpdate(
  val timestamp: Time,
  val targetTagID: Int,
  val robotTTargetTag: Transform3d,
)
