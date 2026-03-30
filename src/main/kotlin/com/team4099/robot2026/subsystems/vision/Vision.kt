package com.team4099.robot2026.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.vision.TimestampedObjectVisionUpdate
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2026.config.constants.Constants
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
import org.team4099.lib.units.base.inSeconds
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

  private var closestTargetTagAcrossCams: Map.Entry<Int, Pair<Int, Transform3d>?>? = null

  var lastTrigVisionUpdate = TimestampedTrigVisionUpdate(Clock.timestamp, -1, Transform3d())

  var objectsDetected: MutableList<MutableList<Translation3d>> =
      MutableList(VisionConstants.OBJECT_CLASS.entries.size) { mutableListOf() }

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
    if (RobotBase.isSimulation() && Constants.Universal.SIMULATE_VISION) {
      visionSim = VisionSystemSim("main")
      visionSim!!.addAprilTags(FieldConstants.fieldLayout)

      cameras.forEach { camera ->
        visionSim!!.addCamera(camera.cameraSim, camera.transform.transform3d)
      }
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
      io[instance].updateInputs(inputs[instance])
      CustomLogger.processInputs("Vision/${io[instance].identifier}", inputs[instance])
      // CustomLogger.recordOutput("Vision/cameraTransform$instance",
      // io[instance].transform.transform3d)
    }

    val visionUpdates = mutableListOf<TimestampedVisionUpdate>()

    val closestTargetingTags = mutableMapOf<Int, Pair<Int, Transform3d>?>()
    for (i in io.indices) {
      closestTargetingTags[i] = null
    }

    for (instance in io.indices) {

      when (io[instance].pipeline) {
        CameraIO.DetectionPipeline.APRIL_TAG -> {
          var targetingTags = mutableListOf<Pair<Int, Transform3d>>()
          var closestTargetTag: Pair<Int, Transform3d>? = null

          var tagTargets = inputs[instance].cameraTargets.filter { it.fiducialId != -1 }

          val cornerData = mutableListOf<Double>()

          for (tag in tagTargets) {
            if (tag.poseAmbiguity < VisionConstants.AMBIGUITY_THESHOLD) {
              if (DriverStation.getAlliance().isPresent) {
                val robotTTag = io[instance].transform.plus(Transform3d(tag.bestCameraToTarget))

                val distanceToTarget = robotTTag.translation.norm

                CustomLogger.recordDebugOutput(
                    "Vision/${io[instance].identifier}/${tag.fiducialId}/robotDistanceToTarget",
                    distanceToTarget.inMeters)

                CustomLogger.recordOutput(
                    "Vision/${io[instance].identifier}/${tag.fiducialId}/robotTTag",
                    robotTTag.transform3d)

                for (corner in tag.detectedCorners) {
                  cornerData.add(corner.x)
                  cornerData.add(corner.y)
                }

                if (tag.fiducialId in tagIDFilter) {
                  targetingTags.add(Pair(tag.fiducialId, robotTTag))
                }
              }

              closestTargetTag = targetingTags.minByOrNull { it.second.translation.norm }

              closestTargetingTags[instance] = closestTargetTag

              CustomLogger.recordDebugOutput(
                  "Vision/${io[instance].identifier}/cornerDetections", cornerData.toDoubleArray())

              CustomLogger.recordOutput(
                  "Vision/${io[instance].identifier}/closestTargetTagID",
                  closestTargetTag?.first ?: -1)

              CustomLogger.recordOutput(
                  "Vision/${io[instance].identifier}/closestTargetTagPose",
                  closestTargetTag?.second?.transform3d ?: Transform3dWPILIB())
            }

            closestTargetTagAcrossCams =
                if (closestTargetingTags[0]?.first != closestTargetingTags[1]?.first) {
                  closestTargetingTags.minByOrNull {
                    it.value?.second?.translation?.norm ?: 1000000.meters
                  }
                } else {
                  mapOf(cameraPreference to closestTargetingTags[cameraPreference]).minByOrNull {
                    it.value?.second?.translation?.norm ?: 1000000.meters
                  }
                }

            CustomLogger.recordOutput(
                "Vision/ClosestTargetTagAcrossAllCams/TagID",
                closestTargetTagAcrossCams?.value?.first ?: -1)

            CustomLogger.recordDebugOutput(
                "Vision/ClosestTargetTagAcrossAllCams/TargetTagPose",
                closestTargetTagAcrossCams?.value?.second?.transform3d ?: Transform3dWPILIB())

            if (closestTargetTagAcrossCams?.key != null &&
                closestTargetTagAcrossCams?.value != null) {
              lastTrigVisionUpdate =
                  TimestampedTrigVisionUpdate(
                      inputs[closestTargetTagAcrossCams?.key ?: 0].timestamp,
                      closestTargetTagAcrossCams?.value?.first ?: -1,
                      Transform3d(
                          closestTargetTagAcrossCams?.value?.second?.translation ?: Translation3d(),
                          closestTargetTagAcrossCams?.value?.second?.rotation ?: Rotation3d()))
            }
          }
        }
        CameraIO.DetectionPipeline.OBJECT_DETECTION -> {
          val objTargets =
              inputs[instance].cameraTargets.filter {
                it.objDetectId != -1 && it.objDetectConf >= VisionConstants.CONFIDENCE_THRESHOLD
              }

          if (RobotBase.isReal()) {
            objectsDetected =
                MutableList(VisionConstants.OBJECT_CLASS.entries.size) { mutableListOf() }

            for (idx in objTargets.indices) {
              // object pose detection credit to 5990 TRIGON Robot Template on Github, available at
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
          } else {
            objectsDetected =
                MutableList(VisionConstants.OBJECT_CLASS.entries.size) { mutableListOf() }

            for (objIdx in VisionConstants.OBJECT_CLASS.entries.toTypedArray().indices) {
              objectsDetected[objIdx].addAll(
                  SimulatedArena.getInstance()
                      .getGamePiecesByType(
                          VisionConstants.OBJECT_CLASS.entries[objIdx].mapleSimType!!)
                      .map {
                        Transform3d(Pose3d(poseSupplier.get()), Pose3d(it.pose3d)).translation
                      })
            }
          }

          for (objects in VisionConstants.OBJECT_CLASS.entries) {
            val closestObject = objectsDetected[objects.id].minByOrNull { it.translation3d.norm }

            if (closestObject != null) {
              lastObjectVisionUpdate[objects.id] =
                  TimestampedObjectVisionUpdate(inputs[instance].timestamp, objects, closestObject)
            }

            CustomLogger.recordOutput(
                "Vision/${io[instance].identifier}/${objects.name}/objectsDetectedPoses",
                *(objectsDetected[objects.id]
                    .map { Pose3d(poseSupplier.get()).plus(Transform3d(it, Rotation3d())).pose3d }
                    .toTypedArray()))

            CustomLogger.recordOutput(
                "Vision/Last${objects.name}VisionUpdate/timestampSeconds",
                lastObjectVisionUpdate[objects.id].timestamp.inSeconds)

            CustomLogger.recordDebugOutput(
                "Vision/Last${objects.name}VisionUpdate/robotTObject",
                lastObjectVisionUpdate[objects.id].robotTObject.translation3d)

            CustomLogger.recordDebugOutput(
                "Vision/Last${objects.name}VisionUpdate/closestObjectPose",
                Pose3d(poseSupplier.get())
                    .plus(
                        Transform3d(lastObjectVisionUpdate[objects.id].robotTObject, Rotation3d()))
                    .pose3d)
          }
        }
      }
    }

    val now = Clock.timestamp

    val tagId0 = closestTargetingTags[0]?.first
    val tagId1 = closestTargetingTags[1]?.first

    val bothSeeingSameTag = tagId0 != null && tagId1 != null && tagId0 == tagId1

    val currentTagId = if (bothSeeingSameTag) tagId0 else null
    val distanceToTag =
        closestTargetTagAcrossCams?.value?.second?.translation?.norm ?: 1000000.meters

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
