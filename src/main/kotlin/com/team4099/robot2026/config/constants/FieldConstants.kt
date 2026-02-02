package com.team4099.robot2026.config.constants

import com.team4099.robot2026.util.AllianceFlipUtil
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import org.ironmaple.simulation.SimulatedArena
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
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

  val ALLIANCE_LINE_X: Length
    get() = AllianceFlipUtil.apply(156.61.inches)

  val TRENCH_LINE_X: Length
    get() = AllianceFlipUtil.apply(181.56.inches)

  val HUB_POSE: Translation3d
    get() = Translation3d(182.11.inches, 158.84.inches, 72.inches)

  val ALLIANCE_ZONE_CENTER: Translation3d
    get() = Translation3d(120.inches, 158.84.inches, 72.inches)

  fun inAllianceZone(pose: Pose3d): Boolean {
    return AllianceFlipUtil.shouldFlip() xor (pose.x < ALLIANCE_LINE_X)
  }

  fun inTrenchAllianceZone(pose: Pose3d): Boolean {
    return AllianceFlipUtil.shouldFlip() xor (pose.x < TRENCH_LINE_X)
  }
}
