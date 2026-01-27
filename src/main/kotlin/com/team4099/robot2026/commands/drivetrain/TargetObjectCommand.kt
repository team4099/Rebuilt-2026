package com.team4099.robot2026.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.ClusterScore
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.PI
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

class TargetObjectCommand(
    private val drivetrain: Drive,
    private val vision: Vision,
    private val targetObjectClass: VisionConstants.OBJECT_CLASS
) : Command() {
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var hasThetaAligned: Boolean = false

  private var startTime: Time = 0.0.seconds

  init {
    addRequirements(drivetrain, vision)

    if (RobotBase.isSimulation()) {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    } else if (DriverStation.isAutonomous()) {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.AUTO_REEF_PID_KP,
              DrivetrainConstants.PID.AUTO_REEF_PID_KI,
              DrivetrainConstants.PID.AUTO_REEF_PID_KD)
    } else {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.OBJECT_ALIGN_KP,
              DrivetrainConstants.PID.OBJECT_ALIGN_KI,
              DrivetrainConstants.PID.OBJECT_ALIGN_KD)
    }
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    startTime = Clock.fpgaTime
    thetaPID.reset()
    hasThetaAligned = false

    if (RobotBase.isSimulation()) {
      val arena = SimulatedArena.getInstance()
      val robotPose = drivetrain.pose.toPose2d()
      val rng = java.util.Random()

      val fuelRadius = 0.15
      val minSpacing = fuelRadius * 2.0 + 0.01 // 1 cm gap
      val maxDistanceFromRobot = 5.0

      data class Point(val x: Double, val y: Double)

      fun dist(a: Point, b: Point): Double = kotlin.math.hypot(a.x - b.x, a.y - b.y)

      val allPlacedPoints = mutableListOf<Point>()

      repeat(3) { clusterIndex ->
        // Random cluster center within 5m of robot
        val r = rng.nextDouble() * maxDistanceFromRobot
        val theta = rng.nextDouble() * 2.0 * Math.PI

        val clusterCenter =
            Point(
                robotPose.x.inMeters + r * kotlin.math.cos(theta),
                robotPose.y.inMeters + r * kotlin.math.sin(theta))

        // Random cluster size (tweak bounds if desired)
        val clusterSize = rng.nextInt(4) + 3 // 3–6 balls

        val clusterPoints = mutableListOf<Point>()

        var attempts = 0
        while (clusterPoints.size < clusterSize && attempts < 500) {
          attempts++

          // Small random offset inside cluster
          val cr = rng.nextDouble() * 0.6
          val ct = rng.nextDouble() * 2.0 * Math.PI

          val candidate =
              Point(
                  clusterCenter.x + cr * kotlin.math.cos(ct),
                  clusterCenter.y + cr * kotlin.math.sin(ct))

          // Ensure no touching ANY existing fuel
          val valid = (clusterPoints + allPlacedPoints).all { dist(it, candidate) >= minSpacing }

          if (valid) {
            clusterPoints.add(candidate)
            allPlacedPoints.add(candidate)
          }
        }

        // Spawn fuel
        clusterPoints.forEach { p ->
          arena.addGamePiece(
              RebuiltFuelOnField(edu.wpi.first.math.geometry.Translation2d(p.x, p.y)))
        }
      }
    }

    CustomLogger.recordOutput("TargetObjectCommand/lastInitialized", Clock.fpgaTime.inSeconds)
  }

  override fun execute() {
    var robotTObject: Translation2d
    val lastUpdate = vision.lastObjectVisionUpdate[targetObjectClass.id]
    if (RobotBase.isSimulation()) {
      var fuelTranslations = vision.objectsDetected[0]
      if (fuelTranslations.isEmpty()) return
      val target =
          ClusterScore.calculateClusterScores(
              drivetrain.pose.toPose2d().pose2d, fuelTranslations.map { it.translation3d })
      CustomLogger.recordOutput("TargetObjectCommand/Target", target)
      robotTObject = Transform2d(drivetrain.pose.toPose2d(), Pose2d(target)).translation
    } else {
      robotTObject = lastUpdate.robotTObject.toTranslation2d()
    }

    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", true)

    val exists = (robotTObject != Translation2d())

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectExists", exists)
    if (!exists || Clock.fpgaTime - lastUpdate.timestamp > .2.seconds) end(interrupted = true)

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectx", robotTObject.x.inMeters)
    CustomLogger.recordOutput("TargetObjectCommand/odomTObjecty", robotTObject.y.inMeters)

    val setpointRotation: Value<Radian> =
        robotTObject.translation2d.angle.radians.radians + drivetrain.pose.rotation.z

    CustomLogger.recordOutput("TargetObjectCommand/setPointRotation", setpointRotation.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/driverot", drivetrain.rotation.z.inDegrees)

    val thetavel =
        thetaPID.calculate(drivetrain.pose.rotation.z, setpointRotation) *
            if (RobotBase.isReal()) -1.0 else 1.0

    CustomLogger.recordOutput("TargetObjectCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("TargetObjectCommand/thetaerror", thetaPID.error.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/hasThetaAligned", hasThetaAligned)

    if ((hasThetaAligned || thetaPID.error.absoluteValue < 8.36.degrees)) {
      hasThetaAligned = true

      drivetrain.runSpeeds(
          ChassisSpeeds(DrivetrainConstants.OBJECT_APPROACH_SPEED, 0.meters.perSecond, thetavel),
          flipIfRed = false)
    } else {
      drivetrain.runSpeeds(
          ChassisSpeeds(0.meters.perSecond, 0.meters.perSecond, thetavel), flipIfRed = false)
    }
  }

  override fun end(interrupted: Boolean) {
    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", false)

    CustomLogger.recordOutput("TargetObjectCommand/interrupted", interrupted)
  }

  override fun isFinished(): Boolean {
    return false
  }
}
