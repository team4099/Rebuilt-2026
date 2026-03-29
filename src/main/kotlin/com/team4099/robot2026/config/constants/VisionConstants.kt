package com.team4099.robot2026.config.constants

import com.team4099.robot2026.config.constants.Constants.WHOAMI
import com.team4099.robot2026.subsystems.vision.camera.CameraIO
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.centi
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond

object VisionConstants {
  val CONTROLLER_RUMBLE_DIST = 2.25.meters

  val BLUE_TARGET_TAGS = arrayOf<Int>()
  val RED_TARGET_TAGS = arrayOf<Int>()

  val AMBIGUITY_THESHOLD = .4
  val CONFIDENCE_THRESHOLD = 0.75
  val TAG_TRUST_THRESHOLD = 0.85

  // Pose acceptance thresholds
  val Z_MINIMUM = -10.centi.meters
  val Z_MAXIMUM = 8.inches + 10.centi.meters
  val POSE_ACCEPTANCE_MAX_LINEAR_SPEED = 3.meters.perSecond
  val POSE_ACCEPTANCE_MAX_ANGULAR_SPEED = 225.degrees.perSecond

  val CAMERAS: Map<String, Pair<CameraIO.DetectionPipeline, Transform3d>>
    get() =
        when (Constants.Universal.whoami) {
          WHOAMI.COMPBOT,
          WHOAMI.ALPHABOT ->
              mapOf(
                  "raven_1" to
                      Pair(
                          CameraIO.DetectionPipeline.APRIL_TAG,
                          Transform3d(
                              Translation3d(9.773802.inches, 11.230054.inches, 8.495934.inches),
                              Rotation3d(0.0.degrees, -28.125.degrees, 30.degrees))),
                  "raven_2" to
                      Pair(
                          CameraIO.DetectionPipeline.APRIL_TAG,
                          Transform3d(
                              Translation3d(9.773802.inches, -11.230054.inches, 8.495934.inches),
                              Rotation3d(0.0.degrees, -28.125.degrees, -30.degrees))),
                  //                  "raven_3" to
                  //                      Pair(
                  //                          CameraIO.DetectionPipeline.APRIL_TAG,
                  //                          Transform3d(
                  //                              Translation3d(13.438976.inches, -3.840252.inches,
                  // 10.189162.inches),
                  //                              Rotation3d(0.degrees, 0.degrees, 0.degrees)))
              )
          WHOAMI.TESTBOT ->
              mapOf(
                  //                  "raven_1" to
                  //                      Pair(
                  //                          CameraIO.DetectionPipeline.APRIL_TAG,
                  //                          Transform3d(
                  //                              Translation3d(10.3.inches, 11.255.inches,
                  // 8.397.inches),
                  //                              Rotation3d(0.0.degrees, -20.degrees,
                  // -30.degrees))),
                  "raven_2" to
                      Pair(
                          CameraIO.DetectionPipeline.APRIL_TAG,
                          Transform3d(
                              Translation3d(10.3.inches, -11.255.inches, 8.397.inches),
                              Rotation3d(0.0.degrees, -20.degrees, 30.degrees))),
              )
          else -> mapOf()
        }

  // x, y, θ
  val singleTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(0.05, 0.05, 9_999.0)
  val multiTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(0.01, 0.01, 0.1)

  enum class OBJECT_CLASS(val id: Int, val mapleSimType: String?) {
    FUEL(0, "Fuel")
  }
}
