package com.team4099.robot2026.config.constants

import com.team4099.robot2026.config.constants.Constants.WHOAMI
import com.team4099.robot2026.subsystems.vision.camera.CameraIO
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N4
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees

object VisionConstants {
  val CONTROLLER_RUMBLE_DIST = 2.25.meters

  val BLUE_TARGET_TAGS = arrayOf<Int>()
  val RED_TARGET_TAGS = arrayOf<Int>()

  val AMBIGUITY_THESHOLD = 1.0
  val XY_STDDEV = 0.05
  val THETA_STDDEV = 10.0

  val CONFIDENCE_THRESHOLD = 0.75
  val TAG_TRUST_THRESHOLD = 0.85

  // Pose acceptance thresholds
  val POSE_FIELD_BOUNDARY_TOLERANCE_METERS = 0.10
  val POSE_ACCEPTANCE_MAX_LINEAR_SPEED_MPS = 6.0
  val POSE_ACCEPTANCE_MAX_ANGULAR_SPEED_RADPS = 12.0

  // Trust rating weights
  val AMBIGUITY_TRUST_RATING = 0.45
  val DISTANCE_TRUST_RATING = 0.35
  val ANGLE_TRUST_RATING = 0.15
  val VELOCITY_TRUST_RATING = 0.15

  // Velocity trust
  val LINEAR_VELOCITY_TRUST_WEIGHT = 0.40
  val ANGULAR_VELOCITY_TRUST_WEIGHT = 0.60

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
                  "raven_3" to
                      Pair(
                          CameraIO.DetectionPipeline.APRIL_TAG,
                          Transform3d(
                              Translation3d(13.438976.inches, -3.840252.inches, 10.189162.inches),
                              Rotation3d(0.degrees, 0.degrees, 0.degrees))))
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
  // TODO tune
  val singleTagStdDevs: Matrix<N4?, N1?> = VecBuilder.fill(8.0, 8.0, 50.0, 99_999.0)
  val multiTagStdDevs: Matrix<N4?, N1?> = VecBuilder.fill(3.0, 3.0, 10.0, 999.0)

  val oldStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(XY_STDDEV, XY_STDDEV, THETA_STDDEV)

  enum class OBJECT_CLASS(val id: Int, val mapleSimType: String?) {
    FUEL(0, "Fuel")
  }
}
