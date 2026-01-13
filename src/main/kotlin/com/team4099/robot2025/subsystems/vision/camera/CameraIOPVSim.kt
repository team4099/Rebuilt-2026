package com.team4099.robot2025.subsystems.vision.camera

import com.team4099.robot2025.config.constants.VisionConstants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.micro
import java.util.function.Supplier

class CameraIOPVSim(
  override val pipeline: CameraIO.DetectionPipeline,
  override val identifier: String,
  override val transform: Transform3d,
  override val poseMeasurementConsumer: (Pose2d?, Double, Matrix<N3?, N1?>) -> Unit = { _, _, _ ->
  },
  override val drivetrainRotationSupplier: Supplier<Angle>
) : CameraIO {
  private val cameraProperties: SimCameraProperties = SimCameraProperties()

  override val photonEstimator: PhotonPoseEstimator =
    PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      transform.transform3d
    )
  override val camera = PhotonCamera(identifier)
  override var cameraSim: PhotonCameraSim? = null
  override var curStdDevs: Matrix<N3?, N1?> = VisionConstants.singleTagStdDevs

  init {
    cameraProperties.setCalibration(1280, 720, 100.0.degrees.inRotation2ds)
    cameraProperties.fps = 45.0

    // examples from documentation
    cameraProperties.setCalibError(0.25, 0.08)
    cameraProperties.exposureTimeMs = 11.3.micro.seconds.inMilliseconds
    cameraProperties.avgLatencyMs = 35.0
    cameraProperties.latencyStdDevMs = 5.0

    cameraSim = PhotonCameraSim(camera, cameraProperties)

    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
  }
}
