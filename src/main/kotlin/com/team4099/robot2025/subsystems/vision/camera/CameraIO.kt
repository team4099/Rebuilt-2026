package com.team4099.robot2025.subsystems.vision.camera

import com.team4099.robot2025.config.constants.VisionConstants
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N4
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import java.util.Optional
import java.util.function.Supplier

interface CameraIO {
  enum class DetectionPipeline {
    APRIL_TAG,
    OBJECT_DETECTION
  }

  val pipeline: DetectionPipeline
  val identifier: String
  val transform: org.team4099.lib.geometry.Transform3d
  val poseMeasurementConsumer: (Pose3dWPILIB?, Double, Matrix<N4?, N1?>) -> Unit
  val drivetrainRotationSupplier: Supplier<Rotation3d>

  val camera: PhotonCamera
  var cameraSim: PhotonCameraSim?
  var curStdDevs: Matrix<N4?, N1?>
  val photonEstimator: PhotonPoseEstimator

  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var usedTargets: List<Int> = listOf<Int>()
    var cameraTargets = mutableListOf<PhotonTrackedTarget>()
    var indices = 0
    var cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), *DoubleArray(9) { 0.0 })
    var distCoeff = MatBuilder.fill(Nat.N8(), Nat.N1(), *DoubleArray(8) { 0.0 })

    override fun toLog(table: LogTable) {
      table.put("timestampSeconds", timestamp.inSeconds)
      table.put("frame", frame.pose3d)
      table.put("usedTargets", usedTargets.toIntArray())
      table.put("cameraMatrix", cameraMatrix.data)
      table.put("distCoeff", distCoeff.data)

      table.put("cameraTargets/indices", cameraTargets.size)

      for (targetIndex in cameraTargets.indices) {
        table.put("cameraTargets/$targetIndex/yaw", cameraTargets[targetIndex].yaw)
        table.put("cameraTargets/$targetIndex/pitch", cameraTargets[targetIndex].pitch)
        table.put("cameraTargets/$targetIndex/area", cameraTargets[targetIndex].area)
        table.put("cameraTargets/$targetIndex/skew", cameraTargets[targetIndex].skew)
        table.put(
          "cameraTargets/$targetIndex/cameraToTarget",
          cameraTargets[targetIndex].bestCameraToTarget
        )

        if (cameraTargets[targetIndex].fiducialId != -1) {
          table.put("cameraTargets/$targetIndex/id", cameraTargets[targetIndex].fiducialId)

          for (i in 0..3) {
            table.put(
              "cameraTargets/$targetIndex/corners/$i",
              cameraTargets[targetIndex].detectedCorners[i]
            )
          }
          table.put(
            "cameraTargets/$targetIndex/ambiguity", cameraTargets[targetIndex].poseAmbiguity
          )
        } else {
          table.put(
            "cameraTargets/$targetIndex/classId",
            cameraTargets[targetIndex].detectedObjectClassID
          )
          table.put(
            "cameraTargets/$targetIndex/confidence", cameraTargets[targetIndex].objDetectConf
          )
        }
      }
    }

    override fun fromLog(table: LogTable) {
      table.get("timestampSeconds", 0.0).let { timestamp = it.seconds }
      table.get("frame", Pose3dWPILIB()).let { frame = Pose3d(it.get(0)) }
      table.get("usedTargets", intArrayOf()).let { usedTargets = it.toList() }

      table.get("cameraTargets/indices", 0).let { indices = it }

      table.get("distCoeff", MatBuilder.fill(Nat.N5(), Nat.N1(), *DoubleArray(5) { 0.0 }).data)
        .let { distCoeff = MatBuilder.fill(Nat.N8(), Nat.N1(), *it) }

      table.get("cameraMatrix", MatBuilder.fill(Nat.N3(), Nat.N3(), *DoubleArray(9) { 0.0 }).data)
        .let { cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), *it) }

      cameraTargets = mutableListOf()

      for (targetID in 0 until indices) {
        val target = PhotonTrackedTarget()

        target.fiducialId = table.get("cameraTargets/$targetID/id", 0)
        target.yaw = table.get("cameraTarget/$targetID/yaw", 0.0)
        target.pitch = table.get("cameraTarget/$targetID/pitch", 0.0)
        target.area = table.get("cameraTarget/$targetID/area", 0.0)
        target.pitch = table.get("cameraTarget/$targetID/skew", 0.0)
        val corners = mutableListOf<TargetCorner>()
        for (i in 1..4) {
          val corner: TargetCorner? = table.get("cameraTarget/$targetID/corners/$i", TargetCorner())
          corners.add(corner ?: TargetCorner())
        }
        target.detectedCorners = corners

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
    if (camera.isConnected) {
      if (camera.cameraMatrix.isPresent) {
        inputs.cameraMatrix = camera.cameraMatrix.get()
      }

      if (camera.distCoeffs.isPresent) {
        inputs.distCoeff = camera.distCoeffs.get()
      }
    }

    val unreadResults = camera.allUnreadResults

    if (unreadResults.isEmpty()) return

    val mostRecentPipelineResult = unreadResults[unreadResults.size - 1]

    inputs.timestamp = mostRecentPipelineResult.timestampSeconds.seconds
    Logger.recordOutput("Vision/$identifier/timestampIG", mostRecentPipelineResult.timestampSeconds)

    inputs.cameraTargets = mutableListOf()

    when (pipeline) {
      DetectionPipeline.APRIL_TAG -> {
        for (result in unreadResults) {
          inputs.cameraTargets.addAll(result.targets)

          if (result.hasTargets()) {
            var visionEst: Optional<EstimatedRobotPose> =
              photonEstimator.estimateCoprocMultiTagPose(result)
            if (visionEst.isEmpty)
              visionEst = photonEstimator.estimatePnpDistanceTrigSolvePose(result)
            if (visionEst.isEmpty) visionEst = photonEstimator.estimateLowestAmbiguityPose(result)

            if (visionEst.isPresent) {
              inputs.usedTargets = visionEst.get().targetsUsed.map { it.fiducialId }

              val poseEst = visionEst.get().estimatedPose
              inputs.frame = Pose3d(poseEst)

              if (result.bestTarget.bestCameraToTarget.translation.norm <
                VisionConstants.FIELD_POSE_RESET_DISTANCE_THRESHOLD.inMeters
              ) {
                updateEstimationStdDevs(visionEst, result.getTargets())

                poseMeasurementConsumer(
                  Pose3dWPILIB(
                    poseEst.x,
                    poseEst.y,
                    poseEst.z,
                    drivetrainRotationSupplier.get().rotation3d
                  ),
                  //                  Pose2d(poseEst2d.x, poseEst.y,
                  // drivetrainRotationSupplier.get().inRotation2ds),
                  visionEst.get().timestampSeconds,
                  curStdDevs
                )
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
        estStdDevs =
          VecBuilder.fill(
            Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
          )
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))
      curStdDevs = estStdDevs
    }
  }
}
