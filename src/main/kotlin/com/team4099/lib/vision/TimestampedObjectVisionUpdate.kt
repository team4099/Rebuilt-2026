package com.team4099.lib.vision

import com.team4099.robot2025.config.constants.VisionConstants
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Time

data class TimestampedObjectVisionUpdate(
  val timestamp: Time,
  val targetClass: VisionConstants.OBJECT_CLASS,
  val robotTObject: Translation2d
)
