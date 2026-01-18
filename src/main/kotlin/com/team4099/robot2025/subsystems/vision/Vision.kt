package com.team4099.robot2025.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TimestampedObjectVisionUpdate
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.vision.camera.CameraIO
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.toPose3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.Logger
import org.photonvision.simulation.VisionSystemSim
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
import java.util.function.Supplier
import edu.wpi.first.math.geometry.Pose3d as WPIPose3d
import edu.wpi.first.math.geometry.Rotation3d as WPIRotation3d

class Vision(vararg cameras: CameraIO, val poseSupplier: Supplier<Pose3d>) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  var tagIDFilter = arrayOf<Int>()

  var isAutoAligning = false
  var isAligned = false

  private val xyStdDev = TunableNumber("Vision/xystdev", VisionConstants.XY_STDDEV)

  private val thetaStdDev = TunableNumber("Vision/thetaStdDev", VisionConstants.THETA_STDDEV)

  private var cameraPreference = 0 // 0 for left 1 for right

  private var closestTargetTagAcrossCams: Map.Entry<Int, Pair<Int, Transform3d>?>? = null

  var lastTrigVisionUpdate = TimestampedTrigVisionUpdate(Clock.fpgaTime, -1, Transform3d())

  var objectsDetected: MutableList<MutableList<Translation3d>> =
    MutableList(VisionConstants.OBJECT_CLASS.values().size) { mutableListOf() }

  var lastObjectVisionUpdate: MutableList<TimestampedObjectVisionUpdate> =
    VisionConstants.OBJECT_CLASS
      .values()
      .map { TimestampedObjectVisionUpdate(Clock.fpgaTime, it, Translation3d()) }
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
    val startTime = Clock.fpgaTime
    visionSim?.update(poseSupplier.get().pose3d)

    Logger.recordOutput(
      "Vision/cameraTransform1",
      edu.wpi.first.math.geometry.Pose3d()
        .transformBy(VisionConstants.CAMERA_TRANSFORMS[0].transform3d)
    )
    Logger.recordOutput(
      "Vision/cameraTransform2",
      edu.wpi.first.math.geometry.Pose3d()
        .transformBy(VisionConstants.CAMERA_TRANSFORMS[1].transform3d)
    )

    Logger.recordOutput("Vision/currentTrigUpdateID", lastTrigVisionUpdate.targetTagID)

    tagIDFilter =
      if (FMSData.isBlue) VisionConstants.BLUE_TARGET_TAGS else VisionConstants.RED_TARGET_TAGS

    for (instance in io.indices) {
      io[instance].updateInputs(inputs[instance])
      Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
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
                if ((tag.fiducialId in VisionConstants.BLUE_TARGET_TAGS && FMSData.isBlue) ||
                  (tag.fiducialId in VisionConstants.RED_TARGET_TAGS && !FMSData.isBlue)
                ) {
                  val fieldTTag =
                    Pose3d(
                      FieldConstants.fieldLayout
                        .getTagPose(tag.fiducialId)
                        .orElse(WPIPose3d(-1337.0, -1337.0, -1337.0, WPIRotation3d.kZero))
                    )

                  val robotTTag =
                    VisionConstants.CAMERA_TRANSFORMS[instance].plus(
                      Transform3d(tag.bestCameraToTarget)
                    )

                  val fieldTRobot = fieldTTag.transformBy(robotTTag.inverse())
                  val distanceToTarget = robotTTag.translation.norm

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/robotDistanceToTarget",
                    distanceToTarget.inMeters
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/robotTTag",
                    robotTTag.transform3d
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/fieldTRobot",
                    fieldTRobot.pose3d
                  )

                  for (corner in tag.detectedCorners) {
                    cornerData.add(corner.x)
                    cornerData.add(corner.y)
                  }

                  if (tag.fiducialId in tagIDFilter) {
                    targetingTags.add(Pair(tag.fiducialId, robotTTag))
                  }
                }
              }

              closestTargetTag = targetingTags.minByOrNull { it.second.translation.norm }

              closestTargetingTags[instance] = closestTargetTag

              Logger.recordOutput(
                "Vision/${VisionConstants.CAMERA_NAMES[instance]}/cornerDetections",
                cornerData.toDoubleArray()
              )

              Logger.recordOutput(
                "Vision/${VisionConstants.CAMERA_NAMES[instance]}/closestTargetTagID",
                closestTargetTag?.first ?: -1
              )

              Logger.recordOutput(
                "Vision/${VisionConstants.CAMERA_NAMES[instance]}/closestTargetTagPose",
                closestTargetTag?.second?.transform3d ?: Transform3dWPILIB()
              )
            }

            Logger.recordOutput(
              "Vision/viewingSameTag",
              closestTargetingTags[0]?.first == closestTargetingTags[1]?.first
            )

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

            Logger.recordOutput(
              "Vision/ClosestTargetTagAcrossAllCams/CameraID",
              if (closestTargetTagAcrossCams != null)
                VisionConstants.CAMERA_NAMES[closestTargetTagAcrossCams?.key ?: -1]
              else "None"
            )

            Logger.recordOutput(
              "Vision/ClosestTargetTagAcrossAllCams/TagID",
              closestTargetTagAcrossCams?.value?.first ?: -1
            )

            Logger.recordOutput(
              "Vision/ClosestTargetTagAcrossAllCams/TargetTagPose",
              closestTargetTagAcrossCams?.value?.second?.transform3d ?: Transform3dWPILIB()
            )

            if (closestTargetTagAcrossCams?.key != null &&
              closestTargetTagAcrossCams?.value != null
            ) {
              lastTrigVisionUpdate =
                TimestampedTrigVisionUpdate(
                  inputs[closestTargetTagAcrossCams?.key ?: 0].timestamp,
                  closestTargetTagAcrossCams?.value?.first ?: -1,
                  Transform3d(
                    closestTargetTagAcrossCams?.value?.second?.translation ?: Translation3d(),
                    closestTargetTagAcrossCams?.value?.second?.rotation ?: Rotation3d()
                  )
                )
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
              MutableList(VisionConstants.OBJECT_CLASS.values().size) { mutableListOf() }

            for (idx in objTargets.indices) {
              // object pose detection credit to 5990 TRIGON Robot Template on Github, available at
              // https://github.com/Programming-TRIGON/RobotTemplate/blob/2d24e98f5e7f5b22657669d6d2a23f5c06f8231d/
              // src/main/java/frc/trigon/robot/misc/objectdetectioncamera/ObjectDetectionCamera.java#L62
              val rotation =
                Rotation3d(
                  0.radians, -objTargets[idx].pitch.degrees, -objTargets[idx].yaw.degrees
                )

              val cameraRotationToObject =
                VisionConstants.CAMERA_TRANSFORMS[instance].toPose3d()
                  .plus(Transform3d(Translation3d(), rotation))
              val xTransform =
                VisionConstants.CAMERA_TRANSFORMS[instance].z /
                  cameraRotationToObject.rotation.y.sin

              val robotTObject =
                cameraRotationToObject.transformBy(
                  Transform3d(Translation3d(xTransform, 0.meters, 0.meters), Rotation3d())
                )

              when (objTargets[idx].detectedObjectClassID) {
                VisionConstants.OBJECT_CLASS.FUEL.id -> {
                  objectsDetected[VisionConstants.OBJECT_CLASS.FUEL.id].add(
                    robotTObject.translation
                  )
                }
              }
            }
          } else {
            objectsDetected =
              MutableList(VisionConstants.OBJECT_CLASS.values().size) { mutableListOf() }

            for (objIdx in VisionConstants.OBJECT_CLASS.values().indices) {
              objectsDetected[objIdx].addAll(
                SimulatedArena.getInstance()
                  .getGamePiecesByType(
                    VisionConstants.OBJECT_CLASS.values()[objIdx].mapleSimType!!
                  )
                  .map { Transform3d(poseSupplier.get(), Pose3d(it.pose3d)).translation }
              )
            }
          }

          for (objects in VisionConstants.OBJECT_CLASS.values()) {
            val closestObject = objectsDetected[objects.id].minByOrNull { it.translation3d.norm }

            if (closestObject != null) {
              lastObjectVisionUpdate[objects.id] =
                TimestampedObjectVisionUpdate(inputs[instance].timestamp, objects, closestObject)
            }

            CustomLogger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${objects.name}/objectsDetectedPoses",
              *(
                objectsDetected[objects.id]
                  .map { poseSupplier.get().plus(Transform3d(it, Rotation3d())).pose3d }
                  .toTypedArray()
                )
            )

            CustomLogger.recordOutput(
              "Vision/Last${objects.name}VisionUpdate/timestampSeconds",
              lastObjectVisionUpdate[objects.id].timestamp.inSeconds
            )

            CustomLogger.recordOutput(
              "Vision/Last${objects.name}VisionUpdate/robotTObject",
              lastObjectVisionUpdate[objects.id].robotTObject.translation3d
            )

            CustomLogger.recordOutput(
              "Vision/Last${objects.name}VisionUpdate/closestObjectPose",
              poseSupplier
                .get()
                .plus(
                  Transform3d(lastObjectVisionUpdate[objects.id].robotTObject, Rotation3d())
                )
                .pose3d
            )
          }
        }
      }
    }

    val now = Clock.fpgaTime

    val tagId0 = closestTargetingTags[0]?.first
    val tagId1 = closestTargetingTags[1]?.first

    val bothSeeingSameTag = tagId0 != null && tagId1 != null && tagId0 == tagId1

    val currentTagId = if (bothSeeingSameTag) tagId0 else null
    val distanceToTag =
      closestTargetTagAcrossCams?.value?.second?.translation?.norm ?: 1000000.meters

    CustomLogger.recordOutput(
      "Vision/rumble",
      currentTagId != null &&
        currentTagId in tagIDFilter &&
        distanceToTag <= VisionConstants.CONTROLLER_RUMBLE_DIST
    )

    if (currentTagId != null &&
      currentTagId in tagIDFilter &&
      distanceToTag <= VisionConstants.CONTROLLER_RUMBLE_DIST
    ) {
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

    Logger.recordOutput(
      "LoggedRobot/Subsystems/VisionLoopTimeMS", (Clock.fpgaTime - startTime).inMilliseconds
    )
  }
}
