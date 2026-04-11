package com.team4099.robot2026.subsystems.superstructure.shooter

import com.ctre.phoenix6.SignalLogger
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.Velocity2d
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat.N1
import edu.wpi.first.math.Nat.N2
import edu.wpi.first.math.Vector
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import java.util.function.Consumer
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.max
import org.team4099.lib.units.min
import org.team4099.lib.units.perSecond

class Shooter(private val io: ShooterIO) : ControlledByStateMachine() {
  val inputs = ShooterIO.ShooterInputs()
  var shooterVoltageTarget: ElectricalPotential = 0.0.volts
    private set

  var shooterVelocityTarget: AngularVelocity = 0.0.degrees.perSecond
    private set

  val isAtTargetedVelocity: Boolean
    get() =
        (currentRequest is Request.ShooterRequest.TargetVelocity &&
            (inputs.shooterLeaderVelocity - shooterVelocityTarget).absoluteValue <
                ShooterConstants.SHOOTER_TOLERANCE)

  var currentState: ShooterState = ShooterState.UNINITIALIZED
  var currentRequest: Request.ShooterRequest = Request.ShooterRequest.Idle()
    set(value) {
      when (value) {
        is Request.ShooterRequest.OpenLoop -> {
          shooterVoltageTarget = value.shooterVoltage
        }
        is Request.ShooterRequest.TargetVelocity -> {
          shooterVelocityTarget = value.targetVelocity
        }
        else -> {}
      }
      field = value
    }

  val shooterTestVel =
      LoggedTunableValue(
          "Shooter/testLaunchSpeedRotPerSec",
          ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY,
          Pair({ it.inRotationsPerSecond }, { it.rotations.perSecond }))

  private val m_sysIdRoutine =
      SysIdRoutine(
          SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              Consumer { state: SysIdRoutineLog.State? ->
                run {
                  SignalLogger.writeString("state", state.toString())
                  CustomLogger.recordOutput("Shooter/sysIdState", state.toString())
                }
              }),
          Mechanism(
              { volts: WPILibVoltage ->
                currentRequest = Request.ShooterRequest.OpenLoop(volts.`in`(Volts).volts)
              },
              null,
              object : SubsystemBase("Shooter") {}))

  init {
    if (RobotBase.isReal()) {
      io.configurePIDCurrent(
          ShooterConstants.PID.REAL_KP0,
          ShooterConstants.PID.REAL_KI0,
          ShooterConstants.PID.REAL_KD0,
          ShooterConstants.PID.REAL_KP1,
          ShooterConstants.PID.REAL_KI1,
          ShooterConstants.PID.REAL_KD1)
      io.configureFFCurrent(
          ShooterConstants.PID.REAL_KS0,
          ShooterConstants.PID.REAL_KV0,
          ShooterConstants.PID.REAL_KA0,
          ShooterConstants.PID.REAL_KS1,
          ShooterConstants.PID.REAL_KV1,
          ShooterConstants.PID.REAL_KA1)
    } else {
      io.configurePIDVoltage(
          ShooterConstants.PID.SIM_KP, ShooterConstants.PID.SIM_KI, ShooterConstants.PID.SIM_KD)
      io.configureFFVoltage(
          ShooterConstants.PID.SIM_KS, ShooterConstants.PID.SIM_KV, ShooterConstants.PID.SIM_KA)
    }
  }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Shooter", inputs)
    CustomLogger.recordOutput(
        "Shooter/targetAngularVelocityRPM", shooterVelocityTarget.inRotationsPerMinute)
    CustomLogger.recordOutput("Shooter/targetVoltage", shooterVoltageTarget.inVolts)
    CustomLogger.recordOutput("Shooter/currentState", currentState)
    CustomLogger.recordOutput("Shooter/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Shooter/isAtTargetedVelocity", isAtTargetedVelocity)

    var nextState = currentState

    when (currentState) {
      ShooterState.UNINITIALIZED -> {
        nextState = fromShooterRequestToState(currentRequest)
      }
      ShooterState.OPEN_LOOP -> {
        io.setVoltage(shooterVoltageTarget)
        nextState = fromShooterRequestToState(currentRequest)
      }
      ShooterState.TARGET_VELOCITY -> {
        io.setVelocity(shooterVelocityTarget)
        nextState = fromShooterRequestToState(currentRequest)
      }
      ShooterState.IDLE -> {
        shooterVelocityTarget = ShooterConstants.VELOCITIES.IDLE_VELOCITY
        io.setVelocity(ShooterConstants.VELOCITIES.IDLE_VELOCITY)
        nextState = fromShooterRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.quasistatic(direction)
  }

  fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.dynamic(direction)
  }

  companion object {
    enum class ShooterState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP,
      TARGET_VELOCITY
    }

    inline fun fromShooterRequestToState(request: Request.ShooterRequest): ShooterState {
      return when (request) {
        is Request.ShooterRequest.TargetVelocity -> ShooterState.TARGET_VELOCITY
        is Request.ShooterRequest.OpenLoop -> ShooterState.OPEN_LOOP
        is Request.ShooterRequest.Idle -> ShooterState.IDLE
      }
    }

    /**
     * Result of a launch velocity calculation.
     *
     * @property distanceToTarget The distance from the robot to the target
     * @property launchVelocity The velocity of the object after being launched
     * @property timeOfFlight The time of flight of the object
     * @property wantedRotation The angle the drivetrain needs to be rotated to face the target.
     */
    data class CalculatedLaunchData(
        val distanceToTarget: Length,
        val launchVelocity: LinearVelocity,
        val timeOfFlight: Time,
        val wantedRotation: Angle
    )

    /**
     * Calculates the required velocity to shoot towards the HUB or the ALLIANCE zone, depending on
     * what's legal based off the current pose.
     *
     * Bases the required velocity off of the momentum provided by the inertia of the drivetrain, as
     * well as drag. A linear feedforward term is calculated based off distance from the adjusted
     * target position.
     *
     * TODO Currently, this assumes the shooter is facing in the +x direction. This can be changed.
     *
     * @param drivetrainPose Instantaneous field-relative pose of the drivetrain
     * @param chassisSpeeds Instantaneous robot-relative speeds of the chassis
     * @return [CalculatedLaunchData] A data class containing the following information about the
     *   trajectory and other information: shooter position, launch velocity on the field plane,
     *   launch velocity in the vertical direction, time of flight, and the desired rotation to aim
     *   in that direction.
     * @see com.team4099.robot2026.commands.drivetrain.AimOTFCommand
     */
    fun calculateLaunchData(
        drivetrainPose: Pose2d,
        chassisSpeeds: ChassisSpeeds
    ): CalculatedLaunchData {
      return calculateLaunchData(
          drivetrainPose,
          chassisSpeeds,
          if (FieldConstants.inTrenchAllianceZone(drivetrainPose)) {
            FieldConstants.HUB_POSE
          } else {
            if (FieldConstants.inLeft(drivetrainPose)) {
              FieldConstants.PASSING_LEFT_TARGET
            } else {
              FieldConstants.PASSING_RIGHT_TARGET
            }
          })
    }

    /**
     * Calculates the required velocity to shoot towards a targeted pose, based off the drivetrain's
     * current position and chassis speeds.
     *
     * Bases the required velocity off of the momentum provided by the inertia of the drivetrain, as
     * well as drag. A linear feedforward term is calculated based off distance from the adjusted
     * target position.
     *
     * TODO Currently, this assumes the shooter is facing in the +x direction. This can be changed.
     *
     * @param drivetrainPose Instantaneous field-relative pose of the drivetrain
     * @param chassisSpeeds Instantaneous robot-relative speeds of the chassis
     * @param targetTranslation The non-adjusted field-relative pose of the target.
     * @return [CalculatedLaunchData] A data class containing the following information about the
     *   trajectory and other information: shooter position, launch velocity on the field plane,
     *   launch velocity in the vertical direction, time of flight, and the desired rotation to aim
     *   in that direction.
     * @see com.team4099.robot2026.commands.drivetrain.AimOTFCommand
     */
    fun calculateLaunchData(
        drivetrainPose: Pose2d,
        chassisSpeeds: ChassisSpeeds,
        targetTranslation: Translation3d
    ): CalculatedLaunchData {
      val rotatedShooter =
          ShooterConstants.SHOOTER_OFFSET.translation.rotateBy(drivetrainPose.rotation)
      val shooterPosition = drivetrainPose.translation + rotatedShooter

      val targetHeight = targetTranslation.z

      // Calculate the shooter's distance to the HUB
      val shooterTTargetX = targetTranslation.x - shooterPosition.x
      val shooterTTargetY = targetTranslation.y - shooterPosition.y
      val shooterTTargetMag =
          sqrt(shooterTTargetX.inMeters.pow(2) + shooterTTargetY.inMeters.pow(2)).meters

      // Get field-relative drivetrain velocity, and convert it into a vector.
      val fieldSpeeds =
          ChassisSpeeds(
              edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
                  chassisSpeeds.chassisSpeedsWPILIB, drivetrainPose.rotation.inRotation2ds))

      // Shooter tangential velocity
      val shooterCurrentTransform =
          ShooterConstants.SHOOTER_OFFSET.translation.rotateBy(drivetrainPose.rotation)
      val shooterSpeeds =
          Velocity2d(
                  (shooterCurrentTransform.x * fieldSpeeds.omega.inRadiansPerSecond +
                          Constants.Universal.EPSILON.meters)
                      .perSecond,
                  (shooterCurrentTransform.y * fieldSpeeds.omega.inRadiansPerSecond +
                          Constants.Universal.EPSILON.meters)
                      .perSecond)
              .rotateBy(90.degrees * fieldSpeeds.omega.sign)

      val driveVector =
          Vector(
              Matrix(
                  N2(),
                  N1(),
                  doubleArrayOf(
                      (fieldSpeeds.vx + shooterSpeeds.x).inMetersPerSecond,
                      (fieldSpeeds.vy + shooterSpeeds.y).inMetersPerSecond)))

      val robotTHubVector =
          Vector(
              Matrix(N2(), N1(), doubleArrayOf(shooterTTargetX.inMeters, shooterTTargetY.inMeters)))

      // Get the distance (signed) between the robot and the HUB
      val hubUnitVector = robotTHubVector.times(1.0 / shooterTTargetMag.inMeters)
      val parallelScalar = driveVector.dot(hubUnitVector).meters

      /**
       * Time for the math...
       *
       * We must solve for v_launch, but that depends on the distance the ball will be displaced by
       * the drivetrain velocity times the time of flight. The time of flight is, unfortunately,
       * also based off of v_launch. We combine the following kinematics equations to derive
       * v_launch independently from time of flight.
       *
       * Variables:
       * ```
       * v_launch = Launch velocity, represented as a number (vector) in 2D.
       *            Multiply by SHOOTER_ANGLE.cos or SHOOTER_ANGLE.sin for
       *            v_launch_x or v_launch_y.
       * h = Height of the hub.
       * s = Height of the shooter.
       * d = Distance shooterTRobot + robotTHub.
       * v_parallel = Movement of drivetrain, parallel to the robotTHub vector.
       * ```
       *
       * From the following kinematics equations:
       * ```
       * d = (v_launch * SHOOTER_ANGLE.cos + v_parallel) * t (1)
       * h = s + (v_launch * SHOOTER_ANGLE.sin) * t + (-g / 2) * t^2 (2)
       * ```
       *
       * Multiply (1) by (v_launch * SHOOTER_ANGLE.sin)
       *
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * d = (v_launch * SHOOTER_ANGLE.sin) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) * t (3)
       * ```
       *
       * From (2):
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * t = (h - s) + (g / 2) * t^2 (4)
       * ```
       *
       * Plug (4) into (3):
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * d = ((h - s) + (g / 2) * t^2) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) (5)
       * ```
       *
       * From (1):
       * ```
       * t = d / (v_launch * SHOOTER_ANGLE.cos + v_parallel) (6)
       * ```
       *
       * Substitute (6) into (5):
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * d = = ((h - s) + (g / 2) * (d / (v_launch * SHOOTER_ANGLE.cos + v_parallel))^2) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)`
       *
       * (v_launch * SHOOTER_ANGLE.sin) * d = (h - s) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) + g * d^2 / (2 * (v_launch * SHOOTER_ANGLE.cos + v_p))` (7)
       * ```
       *
       * Multiply by (v_launch * SHOOTER_ANGLE.cos + v_parallel):
       * ```
       * v_launch * SHOOTER_ANGLE.sin * d * (v_launch * SHOOTER_ANGLE.cos + v_parallel) = (h - s) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)^2 + g * d^2 / 2
       *
       * 0 = (h - s) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)^2 + g * d^2 / 2 - v_launch * SHOOTER_ANGLE.sin * d * (v_launch * SHOOTER_ANGLE.cos + v_parallel)
       *
       * 0 = (h - s) * (v_launch * SHOOTER_ANGLE.cos)^2 + (h - s) * v_parallel^2 + (h - s) * (2 * v_launch * SHOOTER_ANGLE.cos * v_parallel) + g * d^2 / 2 - v_launch^2 * SHOOTER_ANGLE.sin * d * SHOOTER_ANGLE.cos - v_launch * SHOOTER_ANGLE.sin * d * v_parallel
       *
       * 0 =    (h - s) * (SHOOTER_ANGLE.cos)^2 * v_launch^2                (Quadratic term)
       *        - SHOOTER_ANGLE.sin * d * SHOOTER_ANGLE.cos * v_launch^2    (Quadratic term)
       *        + (h - s) * 2 * SHOOTER_ANGLE.cos * v_parallel * v_launch   (Linear term)
       *        - SHOOTER_ANGLE.sin * d * v_parallel * v_launch             (Linear term)
       *        + (h - s) * v_parallel^2 + g * d^2 / 2                      (Constant)
       * ```
       *
       * We know have a quadratic in terms of v_launch.
       *
       * ```
       * A = (h - s) * (SHOOTER_ANGLE.cos)^2 - SHOOTER_ANGLE.sin * SHOOTER_ANGLE.cos * d
       * B = 2 * (h - s) * SHOOTER_ANGLE.cos * v_parallel - SHOOTER_ANGLE.sin * d * v_parallel
       * C = (h - s) * v_parallel^2 + g * d^2 / 2
       * ```
       */
      val a =
          (targetHeight.inMeters - ShooterConstants.SHOOTER_HEIGHT.inMeters) *
              ShooterConstants.SHOOTER_ANGLE.cos.pow(2) -
              ShooterConstants.SHOOTER_ANGLE.sin *
                  ShooterConstants.SHOOTER_ANGLE.cos *
                  shooterTTargetMag.inMeters
      val b =
          2 *
              (targetHeight.inMeters - ShooterConstants.SHOOTER_HEIGHT.inMeters) *
              ShooterConstants.SHOOTER_ANGLE.cos *
              parallelScalar.inMeters -
              ShooterConstants.SHOOTER_ANGLE.sin *
                  shooterTTargetMag.inMeters *
                  parallelScalar.inMeters
      val c =
          (targetHeight.inMeters - ShooterConstants.SHOOTER_HEIGHT.inMeters) *
              parallelScalar.inMeters.pow(2) +
              Constants.Universal.gravity.inMetersPerSecondPerSecond *
                  shooterTTargetMag.inMeters.pow(2) / 2.0

      // To account for things like resistive forces, we add a small
      // feedforward boost proportional to the distance
      val launchSpeedFF = (shooterTTargetMag.inMeters * 0.1).meters.perSecond
      val launchSpeed =
          max(
                  (-b + sqrt(b.pow(2) - 4.0 * a * c)) / (2 * a),
                  (-b - sqrt(b.pow(2) - 4.0 * a * c)) / (2 * a))
              .meters
              .perSecond + launchSpeedFF
      val launchSpeedField = launchSpeed * ShooterConstants.SHOOTER_ANGLE.cos
      val launchSpeedZ = launchSpeed * ShooterConstants.SHOOTER_ANGLE.sin

      val timeOfFlight =
          (shooterTTargetMag.inMeters /
                  (launchSpeed.inMetersPerSecond * ShooterConstants.SHOOTER_ANGLE.cos +
                      parallelScalar.inMeters))
              .seconds

      // The distance the ball travels while in the air due to momentum
      val ballDistanceOffset = driveVector.times(timeOfFlight.inSeconds)

      // todo change comment
      // The wanted rotation is recieved by offsetting the usual angle
      // (the slope connecting the robot and the HUB) by the offset the
      // ball would travel in the air.

      val targetVirt =
          targetTranslation.toTranslation2d() -
              Translation2d(ballDistanceOffset.get(0).meters, ballDistanceOffset.get(1).meters)

      CustomLogger.recordOutput("Shooter/targetVirt", targetVirt.translation2d)

      var theta = drivetrainPose.rotation
      for (i in 1..10) {
        val iterativeShooterPosition =
            drivetrainPose.translation + ShooterConstants.SHOOTER_OFFSET.translation.rotateBy(theta)
        var thetaNew =
            atan2(
                    (targetVirt.y - iterativeShooterPosition.y).inMeters,
                    (targetVirt.x - iterativeShooterPosition.x).inMeters)
                .radians

        // wrap
        //        thetaNew = atan2(thetaNew.sin, thetaNew.cos).radians

        if ((thetaNew - theta).absoluteValue < 1E-3.degrees) {
          theta = thetaNew
          break
        } else {
          theta = thetaNew
        }
      }

      CustomLogger.recordOutput("Shooter/wantedRotDegs", theta.inDegrees)

      return CalculatedLaunchData(
          targetVirt.minus(drivetrainPose.translation).magnitude.meters,
          sqrt(launchSpeedField.inMetersPerSecond.pow(2) + launchSpeedZ.inMetersPerSecond.pow(2))
              .meters
              .perSecond,
          timeOfFlight,
          theta)
    }

    private val distanceToShooterMap: InterpolatingTreeMap<Length, AngularVelocity> =
        InterpolatingTreeMap(
            { startValue, endValue, q ->
              MathUtil.inverseInterpolate(startValue.value, endValue.value, q.value)
            },
            { startValue, endValue, t ->
              AngularVelocity(MathUtil.interpolate(startValue.value, endValue.value, t))
            })

    private val passingShooterMap: InterpolatingTreeMap<Length, AngularVelocity> =
        InterpolatingTreeMap(
            { startValue, endValue, q ->
              MathUtil.inverseInterpolate(startValue.value, endValue.value, q.value)
            },
            { startValue, endValue, t ->
              AngularVelocity(MathUtil.interpolate(startValue.value, endValue.value, t))
            })

    init {
      distanceToShooterMap.put(1.78.meters, 29.rotations.perSecond)
      distanceToShooterMap.put(2.29.meters, 29.5.rotations.perSecond)
      distanceToShooterMap.put(2.54.meters, 32.5.rotations.perSecond)
      distanceToShooterMap.put(2.67.meters, 34.rotations.perSecond)
      distanceToShooterMap.put(3.0.meters, 40.rotations.perSecond)
      distanceToShooterMap.put(3.48.meters, 45.5.rotations.perSecond)
      distanceToShooterMap.put(3.94.meters, 49.5.rotations.perSecond)
      distanceToShooterMap.put(4.07.meters, 50.rotations.perSecond)
      distanceToShooterMap.put(4.34.meters, 50.25.rotations.perSecond)
      distanceToShooterMap.put(4.58.meters, 52.rotations.perSecond)
      distanceToShooterMap.put(4.85.meters, 52.5.rotations.perSecond)
      distanceToShooterMap.put(5.45.meters, 57.rotations.perSecond)
      distanceToShooterMap.put(5.95.meters, 62.5.rotations.perSecond)

      passingShooterMap.put(2.meters, 27.75.rotations.perSecond)
      passingShooterMap.put(2.5.meters, 32.5.rotations.perSecond)
      passingShooterMap.put(3.meters, 37.rotations.perSecond)
      passingShooterMap.put(3.5.meters, 41.6.rotations.perSecond)
      passingShooterMap.put(4.meters, 46.3.rotations.perSecond)
    }

    fun distanceToShooterRPM(distanceToTarget: Length): AngularVelocity {
      if (1.78.meters <= distanceToTarget && distanceToTarget <= 5.9.meters) {
        return distanceToShooterMap.get(distanceToTarget)
      }
      return max(
          ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY,
          min(
              (8.42104 * distanceToTarget.inMeters + 13.25).rotations.perSecond,
              ShooterConstants.VELOCITIES.MAXIMUM_LAUNCH_VELOCITY))
    }

    fun passingDistanceToShooterRPM(distanceToTarget: Length): AngularVelocity {
      if (2.meters <= distanceToTarget && distanceToTarget <= 4.meters) {
        return passingShooterMap.get(distanceToTarget)
      }
      return max(
          ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY,
          min(
              (9.25752 * distanceToTarget.inMeters + 9.25).rotations.perSecond,
              ShooterConstants.VELOCITIES.MAXIMUM_LAUNCH_VELOCITY))
    }
  }
}
