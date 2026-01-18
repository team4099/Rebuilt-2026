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
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import java.util.function.Supplier

class CameraIOPhotonvision(
  override val pipeline: CameraIO.DetectionPipeline,
  override val identifier: String,
  override val transform: Transform3d,
  override val poseMeasurementConsumer: (Pose3d?, Double, Matrix<N4?, N1?>) -> Unit,
  override val drivetrainRotationSupplier: Supplier<Rotation3d>
) : CameraIO {
  override val photonEstimator: PhotonPoseEstimator =
    PhotonPoseEstimator(FieldConstants.fieldLayout, transform.transform3d)
  override val camera: PhotonCamera = PhotonCamera(identifier)
  override var cameraSim: PhotonCameraSim? = null
  override var curStdDevs: Matrix<N4?, N1?> = VisionConstants.singleTagStdDevs

  init {}
}
