package com.team4099.robot2026.subsystems.vision.camera

import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.VisionConstants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import java.util.Optional
import java.util.function.Supplier
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.math.hypot
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle

interface CameraIO {
  enum class DetectionPipeline {
    APRIL_TAG,
    OBJECT_DETECTION
  }

  val pipeline: DetectionPipeline
  val identifier: String
  val transform: org.team4099.lib.geometry.Transform3d
  val poseMeasurementConsumer: (Pose2dWPILIB?, Double, Matrix<N3?, N1?>) -> Unit
  val drivetrainRotationSupplier: Supplier<Angle>
  val drivetrainChassisSpeedsSupplier: Supplier<ChassisSpeeds>

  val camera: PhotonCamera
  var cameraSim: PhotonCameraSim?
  var curStdDevs: Matrix<N3?, N1?>
  val photonEstimator: PhotonPoseEstimator

  fun shouldAcceptPose(
      chassisSpeeds: ChassisSpeeds,
      visionPose: Pose3d,
      ambiguityRating: Double
  ): Boolean {
    if (ambiguityRating > VisionConstants.AMBIGUITY_THESHOLD) return false
    if (visionPose.z < VisionConstants.Z_MINIMUM || visionPose.z > VisionConstants.Z_MAXIMUM)
        return false
    val outOfFieldBounds =
        visionPose.x < 0.meters ||
            visionPose.y < 0.meters ||
            visionPose.x > FieldConstants.fieldLength ||
            visionPose.y > FieldConstants.fieldWidth

    if (outOfFieldBounds) return false

    val linearSpeed = hypot(chassisSpeeds.vx, chassisSpeeds.vy)
    val angularSpeed = chassisSpeeds.omega.absoluteValue

    return linearSpeed <= VisionConstants.POSE_ACCEPTANCE_MAX_LINEAR_SPEED &&
        angularSpeed <= VisionConstants.POSE_ACCEPTANCE_MAX_ANGULAR_SPEED
  }

  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var frameAccepted: Boolean = false
    var cameraTargets = mutableListOf<PhotonTrackedTarget>()
    var indices = 0

    var warmedUp: Boolean = false

    override fun toLog(table: LogTable) {
      table.put("timestampSeconds", timestamp.inSeconds)
      table.put("frame", frame.pose3d)
      table.put("frameAccepted", frameAccepted)

      table.put("cameraTargets/indices", cameraTargets.size)

      for (targetIndex in cameraTargets.indices) {
        table.put("cameraTargets/$targetIndex/yaw", cameraTargets[targetIndex].yaw)
        table.put("cameraTargets/$targetIndex/pitch", cameraTargets[targetIndex].pitch)
        table.put("cameraTargets/$targetIndex/area", cameraTargets[targetIndex].area)
        table.put("cameraTargets/$targetIndex/skew", cameraTargets[targetIndex].skew)
        table.put(
            "cameraTargets/$targetIndex/cameraToTarget",
            cameraTargets[targetIndex].bestCameraToTarget)

        if (cameraTargets[targetIndex].fiducialId != -1) {
          table.put("cameraTargets/$targetIndex/id", cameraTargets[targetIndex].fiducialId)

          table.put(
              "cameraTargets/$targetIndex/ambiguity", cameraTargets[targetIndex].poseAmbiguity)
        } else {
          table.put(
              "cameraTargets/$targetIndex/classId",
              cameraTargets[targetIndex].detectedObjectClassID)
          table.put(
              "cameraTargets/$targetIndex/confidence", cameraTargets[targetIndex].objDetectConf)
        }
      }
    }

    override fun fromLog(table: LogTable) {
      table.get("timestampSeconds", 0.0).let { timestamp = it.seconds }
      table.get("frame", Pose3dWPILIB()).let { frame = Pose3d(it.get(0)) }
      table.get("frameAccepted", false).let { frameAccepted = it }

      table.get("cameraTargets/indices", 0).let { indices = it }

      cameraTargets = mutableListOf()

      for (targetID in 0..indices) {
        val target = PhotonTrackedTarget()

        target.fiducialId = table.get("cameraTargets/$targetID/id", 0)
        target.yaw = table.get("cameraTarget/$targetID/yaw", 0.0)
        target.pitch = table.get("cameraTarget/$targetID/pitch", 0.0)
        target.area = table.get("cameraTarget/$targetID/area", 0.0)
        target.pitch = table.get("cameraTarget/$targetID/skew", 0.0)

        target.bestCameraToTarget =
            table.get("cameraTarget/$targetID/cameraToTarget", Transform3d())?.get(0)
                ?: Transform3d()
        target.poseAmbiguity = table.get("cameraTarget/$targetID/ambiguity", 0.0)

        cameraTargets.add(target)
      }
    }
  }

  // note(nathan): pv and pvsim use same exact logic so i put it in io
  fun updateInputs(inputs: CameraInputs) {
    val unreadResults = camera.allUnreadResults

    if (unreadResults.isEmpty()) return

    val mostRecentPipelineResult = unreadResults.last()

    inputs.timestamp = mostRecentPipelineResult.timestampSeconds.seconds
    Logger.recordOutput("Vision/$identifier/timestampIG", mostRecentPipelineResult.timestampSeconds)

    inputs.cameraTargets = mutableListOf()

    if (!inputs.warmedUp) {
      inputs.cameraTargets.addAll(
          FieldConstants.fieldLayout.tags.map {
            PhotonTrackedTarget(
                0.0,
                0.0,
                0.0,
                0.0,
                it.ID,
                0,
                0.0f,
                Transform3dWPILIB(),
                Transform3dWPILIB(),
                0.0,
                mutableListOf(TargetCorner()),
                mutableListOf(TargetCorner()))
          })

      inputs.warmedUp = true
      return
    }

    when (pipeline) {
      DetectionPipeline.APRIL_TAG -> {
        for (result in unreadResults) {
          inputs.cameraTargets.addAll(result.targets)

          if (result.hasTargets()) {
            var visionEst: Optional<EstimatedRobotPose> =
                photonEstimator.estimateCoprocMultiTagPose(result)
            if (visionEst.isEmpty) visionEst = photonEstimator.estimateLowestAmbiguityPose(result)

            if (visionEst.isPresent) {
              val poseEst = visionEst.get().estimatedPose
              inputs.frame = Pose3d(poseEst)

              val avgAmbiguityAcrossTargets =
                  visionEst.get().targetsUsed.map { it.poseAmbiguity }.toTypedArray().average()

              inputs.frameAccepted =
                  shouldAcceptPose(
                      drivetrainChassisSpeedsSupplier.get(),
                      inputs.frame,
                      avgAmbiguityAcrossTargets)

              if (inputs.frameAccepted) {
                updateEstimationStdDevs(visionEst, result.getTargets())

                poseMeasurementConsumer(
                    poseEst.toPose2d(), visionEst.get().timestampSeconds, curStdDevs)
              }
            }
          }
        }
      }
      DetectionPipeline.OBJECT_DETECTION -> {}
    }
  }

  // from documentation
  fun updateEstimationStdDevs(
      estimatedPose: Optional<EstimatedRobotPose>?,
      targets: MutableList<PhotonTrackedTarget>
  ) {
    if (estimatedPose == null || estimatedPose.isEmpty) {
      curStdDevs = VisionConstants.singleTagStdDevs
      return
    }
    var estStdDevs = VisionConstants.singleTagStdDevs
    var numTags = 0
    var avgDist = 0.0

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (tgt in targets) {
      val tagPose = photonEstimator.fieldTags.getTagPose(tgt.getFiducialId())
      if (tagPose.isEmpty) continue
      numTags++
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .translation
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().translation)
    }

    if (numTags == 0) {
      curStdDevs = VisionConstants.singleTagStdDevs
    } else {
      // One or more tags visible, run the full heuristic.
      avgDist /= numTags.toDouble()
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionConstants.multiTagStdDevs
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))
      curStdDevs = estStdDevs
    }
  }
}
