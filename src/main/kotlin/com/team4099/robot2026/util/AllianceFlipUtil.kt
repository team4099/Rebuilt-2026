package com.team4099.robot2026.util

import com.team4099.lib.math.Zone2d
import com.team4099.robot2026.config.constants.FieldConstants
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation2dWPILIB
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.sin
import org.wpilib.driverstation.Alliance
import org.wpilib.driverstation.MatchState
import org.wpilib.math.geometry.Rotation2d

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in [FieldConstants] are stored with the origin at the rightmost point on the blue alliance
 * wall.
 */
object AllianceFlipUtil {
  /** Flips a translation to the correct side of the field based on the current alliance color. */
  fun apply(translation: Translation2d, force: Boolean = false): Translation2d {
    return if (shouldFlip() || force) {
      Translation2d(
          FieldConstants.fieldLength - translation.x, FieldConstants.fieldWidth - translation.y)
    } else {
      translation
    }
  }

  fun apply(translation: Translation2dWPILIB): Translation2dWPILIB {
    return if (shouldFlip()) {
      Translation2dWPILIB(
          FieldConstants.fieldLength.inMeters - translation.x,
          FieldConstants.fieldWidth.inMeters - translation.y)
    } else {
      translation
    }
  }

  fun apply(translation: Translation3d): Translation3d {
    val appliedTranslation = apply(translation.toTranslation2d())
    return Translation3d(appliedTranslation.x, appliedTranslation.y, translation.z)
  }

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  fun apply(xCoordinate: Length): Length {
    return if (shouldFlip()) {
      FieldConstants.fieldLength - xCoordinate
    } else {
      xCoordinate
    }
  }

  /** Flips a rotation based on the current alliance color. */
  fun apply(rotation: Rotation2d): Rotation2d {
    return if (shouldFlip()) {
      Rotation2d(-rotation.cos, rotation.sin)
    } else {
      rotation
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  fun apply(
      pose: Pose2d
  ): Pose2d { // flipToRed is a hacky way to get it to flip when the driver station is empty
    return if (shouldFlip()) {
      Pose2d(
          FieldConstants.fieldLength - pose.x,
          FieldConstants.fieldWidth - pose.y,
          Angle(-pose.rotation.cos, -pose.rotation.sin))
    } else {
      pose
    }
  }

  fun apply(zone: Zone2d, force: Boolean = false): Zone2d {
    return if (shouldFlip() || force) {
      Zone2d(zone.vertices.map { AllianceFlipUtil.apply(it, force) })
    } else {
      zone
    }
  }

  fun shouldFlip(): Boolean {
    if (MatchState.getAlliance().isPresent) {
      return MatchState.getAlliance().get() == Alliance.RED
    } else {
      return false
    }
  }
}
