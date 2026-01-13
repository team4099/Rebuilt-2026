package com.team4099.lib.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.Time

/** Represents a single vision pose with a timestamp and associated standard deviations. */
data class TimestampedVisionUpdate(
  val timestamp: Time,
  val fieldTRobot: Pose2d,
  val stdDevs: Matrix<N3, N1>,
  val fromVision: Boolean = false
)
