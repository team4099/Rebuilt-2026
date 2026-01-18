package com.team4099.robot2025.config.constants

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import org.ironmaple.simulation.SimulatedArena
import org.team4099.lib.units.base.meters

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. **All units in Meters** <br></br> <br></br>
 *
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE
 * wall.<br></br> <br></br> Length refers to the *x* direction (as described by wpilib) <br></br>
 * Width refers to the *y* direction (as described by wpilib)
 */
object FieldConstants {
  val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)

  val fieldLength = fieldLayout.fieldLength.meters
  val fieldWidth = fieldLayout.fieldWidth.meters

  val EMPTY_MAPLESIM_FIELD =
    object : SimulatedArena(object : FieldMap() {}) {
      override fun placeGamePiecesOnField() {}
    }
}
