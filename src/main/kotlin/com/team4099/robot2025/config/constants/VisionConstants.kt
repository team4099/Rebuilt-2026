package com.team4099.robot2025.config.constants

import com.team4099.robot2025.util.AllianceFlipUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import java.util.function.Supplier

object VisionConstants {
  const val SIM_POSE_TOPIC_NAME = "Odometry/groundTruthPose"
  const val POSE_TOPIC_NAME = "Odometry/pose"

  val CONTROLLER_RUMBLE_DIST = 2.25.meters

  const val NUM_OF_CAMERAS = 3

  val BLUE_TARGET_TAGS = arrayOf<Int>()
  val RED_TARGET_TAGS = arrayOf<Int>()


  val AMBIGUITY_THESHOLD = 1.0
  val XY_STDDEV = 0.05
  val THETA_STDDEV = 10.0

  val CONFIDENCE_THRESHOLD = 0.75

  val CAMERA_TRANSFORMS =
    listOf(
      Transform3d(
        Translation3d(-10.398.inches, -11.593.inches, 8.36.inches),
        Rotation3d(0.0.degrees, -20.degrees, 145.degrees)
      ), // raven_1
      Transform3d(
        Translation3d(-10.398.inches, 11.593.inches, 8.36.inches),
        Rotation3d(0.0.degrees, -20.degrees, -145.degrees)
      ), // raven_2
      Transform3d(
        Translation3d(10.398.inches, -11.593.inches, 8.36.inches)
          .plus(Translation3d(4.342.inches, 18.23.inches, 32.372.inches)),
        Rotation3d(0.0.degrees, 30.degrees, 0.degrees)
      ) // raven_o1
    )

  val CAMERA_NAMES = listOf("raven_1", "raven_2", "raven_o1")

  // x, y, θ
  // TODO tune
  val singleTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(4.0, 4.0, 10.0)
  val multiTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(0.5, 0.5, 7.0)

  val oldStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(XY_STDDEV, XY_STDDEV, THETA_STDDEV)

  val FIELD_POSE_RESET_DISTANCE_THRESHOLD = .75.meters

  enum class OBJECT_CLASS(val id: Int, val mapleSimType: String?) {
    ALGAE(0, "Algae"),
    CORAL(1, "Coral")
  }
}
