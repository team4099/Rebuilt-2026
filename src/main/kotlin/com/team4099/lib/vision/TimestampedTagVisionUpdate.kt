package com.team4099.lib.vision

import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time

data class TimestampedTagVisionUpdate(
    val timestamp: Time,
    val targetTagID: Int,
    val robotTTargetTag: Transform3d,
    val estimatedPose: Pose3d,
)
