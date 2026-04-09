package com.team4099.robot2026.subsystems.vision.camera

import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.VisionConstants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import java.util.function.Supplier
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.derived.Angle

class CameraIOPhotonvision(
    override val pipeline: CameraIO.DetectionPipeline,
    override val identifier: String,
    override val transform: Transform3d,
    override val poseMeasurementConsumer: (Pose2d?, Double, Matrix<N3?, N1?>) -> Unit,
    override val drivetrainRotationSupplier: Supplier<Angle>,
    override val drivetrainChassisSpeedsSupplier: Supplier<ChassisSpeeds>
) : CameraIO {
  override val photonEstimator: PhotonPoseEstimator =
      PhotonPoseEstimator(FieldConstants.fieldLayout, transform.transform3d)
  override val camera: PhotonCamera = PhotonCamera(identifier)
  override var cameraSim: PhotonCameraSim? = null
  override var curStdDevs: Matrix<N3?, N1?> = VisionConstants.singleTagStdDevs

  init {
    photonEstimator.estimateLowestAmbiguityPose(
        PhotonPipelineResult(
            0,
            0,
            0,
            0,
            listOf(
                PhotonTrackedTarget(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1,
                    0,
                    0.0f,
                    Transform3dWPILIB(),
                    Transform3dWPILIB(),
                    0.0,
                    mutableListOf(TargetCorner()),
                    mutableListOf(TargetCorner())))))
  }
}
