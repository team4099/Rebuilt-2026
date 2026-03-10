package com.team4099.robot2026.subsystems.vision.camera

import com.team4099.robot2026.config.constants.VisionConstants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
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
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond

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

  fun calculateTagTrustScore(
      tag: PhotonTrackedTarget,
      distanceToTarget: Double,
      robotTTag: org.team4099.lib.geometry.Transform3d,
      chassisSpeeds: org.team4099.lib.kinematics.ChassisSpeeds
  ): Double {
    // 1. Ambiguity trust (0-1, higher is better)
    val ambiguityTrust = (1.0 - tag.poseAmbiguity).coerceIn(0.0, 1.0)

    // 2. Distance trust (0-1, closer is better)
    val distanceTrust =
        when {
          distanceToTarget <= 2.0 -> 1.0
          distanceToTarget <= 4.0 -> 1.0 - ((distanceToTarget - 2.0) / 2.0) * 0.3
          distanceToTarget <= 6.0 -> 0.7 - ((distanceToTarget - 4.0) / 2.0) * 0.35
          else -> 0.35
        }

    // 3. Angle trust
    val angleToTag = kotlin.math.abs(kotlin.math.atan2(robotTTag.y.inMeters, robotTTag.x.inMeters))
    val angleTrust =
        when {
          angleToTag <= kotlin.math.PI / 6 -> 1.0
          angleToTag <= kotlin.math.PI / 3 ->
              1.0 - ((angleToTag - kotlin.math.PI / 6) / (kotlin.math.PI / 6)) * 0.3
          angleToTag <= kotlin.math.PI / 2 ->
              0.7 - ((angleToTag - kotlin.math.PI / 3) / (kotlin.math.PI / 6)) * 0.4
          else -> 0.3
        }

    // 4. Drivetrain velocity trust (slower is better)
    val linearVelocity =
        kotlin.math.hypot(chassisSpeeds.vx.inMetersPerSecond, chassisSpeeds.vy.inMetersPerSecond)
    val angularVelocity = kotlin.math.abs(chassisSpeeds.omega.inRadiansPerSecond)

    val linearVelocityTrust =
        when {
          linearVelocity <= 0.5 -> 1.0
          linearVelocity <= 2.0 -> 1.0 - ((linearVelocity - 0.5) / 1.5) * 0.4
          linearVelocity <= 4.0 -> 0.6 - ((linearVelocity - 2.0) / 2.0) * 0.3
          else -> 0.3
        }

    val angularVelocityTrust =
        when {
          angularVelocity <= 0.5 -> 1.0
          angularVelocity <= 2.0 -> 1.0 - ((angularVelocity - 0.5) / 1.5) * 0.5
          angularVelocity <= 4.0 -> 0.5 - ((angularVelocity - 2.0) / 2.0) * 0.35
          else -> 0.15
        }

    val velocityTrust =
        (linearVelocityTrust * VisionConstants.LINEAR_VELOCITY_TRUST_WEIGHT) +
            (angularVelocityTrust * VisionConstants.ANGULAR_VELOCITY_TRUST_WEIGHT)

    val weightedTrust =
        (ambiguityTrust * VisionConstants.AMBIGUITY_TRUST_RATING) +
            (distanceTrust * VisionConstants.DISTANCE_TRUST_RATING) +
            (angleTrust * VisionConstants.ANGLE_TRUST_RATING) +
            (velocityTrust * VisionConstants.VELOCITY_TRUST_RATING)

    return weightedTrust.coerceIn(0.0, 1.0)
  }

  fun calculateTagTrust(
      tag: PhotonTrackedTarget,
      distanceToTarget: Double,
      robotTTag: org.team4099.lib.geometry.Transform3d,
      chassisSpeeds: org.team4099.lib.kinematics.ChassisSpeeds,
      minTrustThreshold: Double = VisionConstants.TAG_TRUST_THRESHOLD
  ): Boolean {
    return calculateTagTrustScore(tag, distanceToTarget, robotTTag, chassisSpeeds) >=
        minTrustThreshold
  }

  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var cameraTargets = mutableListOf<PhotonTrackedTarget>()
    var indices = 0

    override fun toLog(table: LogTable) {
      table.put("timestampSeconds", timestamp.inSeconds)
      table.put("frame", frame.pose3d)

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
              val poseEst = visionEst.get().estimatedPose
              inputs.frame = Pose3d(poseEst)

              updateEstimationStdDevs(visionEst, result.getTargets())

              poseMeasurementConsumer(poseEst, visionEst.get().timestampSeconds, curStdDevs)
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
                  Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))
      curStdDevs = estStdDevs
    }
  }
}
