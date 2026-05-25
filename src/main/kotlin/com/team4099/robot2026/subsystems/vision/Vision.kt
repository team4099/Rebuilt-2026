package com.team4099.robot2026.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.vision.TimestampedObjectVisionUpdate
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.subsystems.vision.camera.CameraIO
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.toPose3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier
import org.ironmaple.simulation.SimulatedArena
import org.photonvision.simulation.VisionSystemSim
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin

class Vision(vararg cameras: CameraIO, val poseSupplier: Supplier<Pose2d>) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  var tagIDFilter = arrayOf<Int>()

  var isAutoAligning = false
  var isAligned = false

  private var cameraPreference = 0 // 0 for left 1 for right

  private val closestTargetingTags: Array<Pair<Int, Transform3d>?> = arrayOfNulls(cameras.size)

  var lastTrigVisionUpdate = TimestampedTrigVisionUpdate(Clock.timestamp, -1, Transform3d())

  var objectsDetected: Array<MutableList<Translation3d>> =
      Array(VisionConstants.OBJECT_CLASS.entries.size) { mutableListOf() }

  var lastObjectVisionUpdate: MutableList<TimestampedObjectVisionUpdate> =
      VisionConstants.OBJECT_CLASS.entries
          .map { TimestampedObjectVisionUpdate(Clock.timestamp, it, Translation3d()) }
          .toMutableList()

  private var lastSeenTagId: Int? = null
  private var pulseEndTime = 0.0.seconds
  var autoAlignReadyRumble = false
    private set

  private var visionSim: VisionSystemSim? = null

  init {
    if (RobotBase.isSimulation()) {
      visionSim = VisionSystemSim("main")
      visionSim!!.addAprilTags(FieldConstants.fieldLayout)

      cameras.forEach { camera ->
        visionSim!!.addCamera(camera.cameraSim, camera.transform.transform3d)
      }
    }

    for (identifier in cameras.map { it.identifier }) {
      for (tag in FieldConstants.fieldLayout.tags) {
        CustomLogger.recordDebugOutput("Vision/$identifier/${tag.ID}/robotDistanceToTarget", -1.0)
        CustomLogger.recordDebugOutput(
            "Vision/$identifier/${tag.ID}/robotTTag", Transform3dWPILIB())
      }
      CustomLogger.recordDebugOutput("Vision/$identifier/cornerDetections", DoubleArray(0))
      CustomLogger.recordDebugOutput("Vision/$identifier/closestTargetTagID", -1)
      CustomLogger.recordDebugOutput("Vision/$identifier/closestTargetTagPose", Transform3dWPILIB())
    }
  }

  override fun periodic() {
    val startTime = Clock.epochTime
    visionSim?.update(poseSupplier.get().pose2d)

    tagIDFilter =
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
            DriverStation.Alliance.Blue)
            VisionConstants.BLUE_TARGET_TAGS
        else VisionConstants.RED_TARGET_TAGS

    for (instance in io.indices) {
      val ioStartTime = Clock.epochTime
      io[instance].updateInputs(inputs[instance])
      CustomLogger.processInputs("Vision/${io[instance].identifier}", inputs[instance])
      CustomLogger.recordOutput(
          "Vision/${instance}IOLoopTimeMS", (Clock.epochTime - ioStartTime).inMilliseconds)
    }

    for (i in io.indices) {
      closestTargetingTags[i] = null
    }

    for (instance in io.indices) {
      val instanceStartTime = Clock.epochTime

      when (io[instance].pipeline) {
        CameraIO.DetectionPipeline.APRIL_TAG -> {
          var closestTargetTag: Pair<Int, Transform3d>? = null
          var minNorm = 1000000.meters

          val tagTargets = inputs[instance].cameraTargets

          for (i in tagTargets.indices) {
            val tag = tagTargets[i]
            if (tag.fiducialId != -1 && tag.poseAmbiguity < VisionConstants.AMBIGUITY_THESHOLD) {
              val robotTTag = io[instance].transform.plus(Transform3d(tag.bestCameraToTarget))

              val distanceToTarget = robotTTag.translation.norm

              CustomLogger.recordDebugOutput(
                  "Vision/${io[instance].identifier}/${tag.fiducialId}/robotDistanceToTarget",
                  distanceToTarget.inMeters)

              CustomLogger.recordDebugOutput(
                  "Vision/${io[instance].identifier}/${tag.fiducialId}/robotTTag",
                  robotTTag.transform3d)

              if (tag.fiducialId in tagIDFilter) {
                if (distanceToTarget < minNorm) {
                  minNorm = distanceToTarget
                  closestTargetTag = Pair(tag.fiducialId, robotTTag)
                }
              }
            }
          }

          closestTargetingTags[instance] = closestTargetTag

          CustomLogger.recordDebugOutput(
              "Vision/${io[instance].identifier}/closestTargetTagID", closestTargetTag?.first ?: -1)

          CustomLogger.recordDebugOutput(
              "Vision/${io[instance].identifier}/closestTargetTagPose",
              closestTargetTag?.second?.transform3d ?: Transform3dWPILIB())
        }
        CameraIO.DetectionPipeline.OBJECT_DETECTION -> {
          val objTargets = inputs[instance].cameraTargets

          if (RobotBase.isReal()) {
            for (list in objectsDetected) {
              list.clear()
            }

            for (idx in objTargets.indices) {
              if (objTargets[idx].objDetectId != -1 &&
                  objTargets[idx].objDetectConf >= VisionConstants.CONFIDENCE_THRESHOLD) {
                // object pose detection credit to 5990 TRIGON Robot Template on Github, available
                // at
                // https://github.com/Programming-TRIGON/RobotTemplate/blob/2d24e98f5e7f5b22657669d6d2a23f5c06f8231d/
                // src/main/java/frc/trigon/robot/misc/objectdetectioncamera/ObjectDetectionCamera.java#L62
                val rotation =
                    Rotation3d(
                        0.radians, -objTargets[idx].pitch.degrees, -objTargets[idx].yaw.degrees)

                val cameraRotationToObject =
                    io[instance].transform.toPose3d().plus(Transform3d(Translation3d(), rotation))
                val xTransform = io[instance].transform.z / cameraRotationToObject.rotation.y.sin

                val robotTObject =
                    cameraRotationToObject.transformBy(
                        Transform3d(Translation3d(xTransform, 0.meters, 0.meters), Rotation3d()))

                when (objTargets[idx].detectedObjectClassID) {
                  VisionConstants.OBJECT_CLASS.FUEL.id -> {
                    objectsDetected[VisionConstants.OBJECT_CLASS.FUEL.id].add(
                        robotTObject.translation)
                  }
                }
              }
            }
          } else {
            for (list in objectsDetected) {
              list.clear()
            }

            for (objIdx in VisionConstants.OBJECT_CLASS.entries.toTypedArray().indices) {
              val gamePieces =
                  SimulatedArena.getInstance()
                      .getGamePiecesByType(
                          VisionConstants.OBJECT_CLASS.entries[objIdx].mapleSimType!!)
              for (j in gamePieces.indices) {
                objectsDetected[objIdx].add(
                    Transform3d(Pose3d(poseSupplier.get()), Pose3d(gamePieces[j].pose3d))
                        .translation)
              }
            }
          }

          for (objects in VisionConstants.OBJECT_CLASS.entries) {
            var minNorm = Double.MAX_VALUE
            var closestObject: Translation3d? = null
            val objList = objectsDetected[objects.id]

            for (j in objList.indices) {
              val norm = objList[j].translation3d.norm
              if (norm < minNorm) {
                minNorm = norm
                closestObject = objList[j]
              }
            }

            if (closestObject != null) {
              lastObjectVisionUpdate[objects.id] =
                  TimestampedObjectVisionUpdate(inputs[instance].timestamp, objects, closestObject)
            }

            //            CustomLogger.recordOutput(
            //
            // "Vision/${io[instance].identifier}/${objects.name}/objectsDetectedPoses",
            //                *(objectsDetected[objects.id]
            //                    .map { Pose3d(poseSupplier.get()).plus(Transform3d(it,
            // Rotation3d())).pose3d }
            //                    .toTypedArray()))
            //
            //            CustomLogger.recordOutput(
            //                "Vision/Last${objects.name}VisionUpdate/timestampSeconds",
            //                lastObjectVisionUpdate[objects.id].timestamp.inSeconds)
            //
            //            CustomLogger.recordDebugOutput(
            //                "Vision/Last${objects.name}VisionUpdate/robotTObject",
            //                lastObjectVisionUpdate[objects.id].robotTObject.translation3d)
            //
            //            CustomLogger.recordDebugOutput(
            //                "Vision/Last${objects.name}VisionUpdate/closestObjectPose",
            //                Pose3d(poseSupplier.get())
            //                    .plus(
            //                        Transform3d(lastObjectVisionUpdate[objects.id].robotTObject,
            // Rotation3d()))
            //                    .pose3d)
          }
        }
      }
    }

    val now = Clock.timestamp

    var bestCamIndex = -1
    var bestNorm = 1000000.meters
    var bestTagId = -1
    var bestTransform: Transform3d? = null

    if (io.size >= 2 &&
        closestTargetingTags[0]?.first != null &&
        closestTargetingTags[0]?.first == closestTargetingTags[1]?.first) {
      val pref = closestTargetingTags[cameraPreference]
      if (pref != null) {
        bestCamIndex = cameraPreference
        bestNorm = pref.second.translation.norm
        bestTagId = pref.first
        bestTransform = pref.second
      }
    } else {
      for (i in closestTargetingTags.indices) {
        val tag = closestTargetingTags[i]
        if (tag != null) {
          val norm = tag.second.translation.norm
          if (norm < bestNorm) {
            bestNorm = norm
            bestCamIndex = i
            bestTagId = tag.first
            bestTransform = tag.second
          }
        }
      }
    }

    CustomLogger.recordDebugOutput("Vision/ClosestTargetTagAcrossAllCams/TagID", bestTagId)

    CustomLogger.recordDebugOutput(
        "Vision/ClosestTargetTagAcrossAllCams/TargetTagPose",
        bestTransform?.transform3d ?: Transform3dWPILIB())

    if (bestCamIndex != -1 && bestTransform != null) {
      lastTrigVisionUpdate =
          TimestampedTrigVisionUpdate(
              inputs[bestCamIndex].timestamp,
              bestTagId,
              Transform3d(bestTransform.translation, bestTransform.rotation))
    }

    val tagId0 = closestTargetingTags[0]?.first
    val tagId1 = closestTargetingTags[1]?.first

    val bothSeeingSameTag = tagId0 != null && tagId1 != null && tagId0 == tagId1

    val currentTagId = if (bothSeeingSameTag) tagId0 else null
    val distanceToTag = bestNorm

    if (currentTagId != null &&
        currentTagId in tagIDFilter &&
        distanceToTag <= VisionConstants.CONTROLLER_RUMBLE_DIST) {
      if (lastSeenTagId == null || currentTagId != lastSeenTagId) {
        pulseEndTime = now + 0.25.seconds
        autoAlignReadyRumble = true
      }
      lastSeenTagId = currentTagId
    } else if (currentTagId == null) {
      lastSeenTagId = null
    }

    if (now > pulseEndTime) {
      autoAlignReadyRumble = false
    }

    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/VisionLoopTimeMS", (Clock.epochTime - startTime).inMilliseconds)
  }
}
