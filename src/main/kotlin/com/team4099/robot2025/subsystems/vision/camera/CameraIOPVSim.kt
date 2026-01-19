package com.team4099.robot2025.subsystems.vision.camera

import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.VisionConstants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.micro
import java.util.function.Supplier

class CameraIOPVSim(
  override val pipeline: CameraIO.DetectionPipeline,
  override val identifier: String,
  override val transform: Transform3d,
  override val poseMeasurementConsumer: (Pose3d?, Double, Matrix<N4?, N1?>) -> Unit = { _, _, _ ->
  },
  override val drivetrainRotationSupplier: Supplier<Rotation3d>
) : CameraIO {
  private val cameraProperties: SimCameraProperties = SimCameraProperties()

  override val photonEstimator: PhotonPoseEstimator =
    PhotonPoseEstimator(FieldConstants.fieldLayout, transform.transform3d)
  override val camera = PhotonCamera(identifier)
  override var cameraSim: PhotonCameraSim? = null
  override var curStdDevs: Matrix<N4?, N1?> = VisionConstants.singleTagStdDevs

  init {
    cameraProperties.setCalibration(1280, 720, 100.0.degrees.inRotation2ds)
    cameraProperties.fps = 45.0

    // examples from documentation
    cameraProperties.setCalibError(0.25, 0.08)
    cameraProperties.exposureTimeMs = 11.3.micro.seconds.inMilliseconds
    cameraProperties.avgLatencyMs = 35.0
    cameraProperties.latencyStdDevMs = 5.0

    cameraSim = PhotonCameraSim(camera, cameraProperties)
  }
}
