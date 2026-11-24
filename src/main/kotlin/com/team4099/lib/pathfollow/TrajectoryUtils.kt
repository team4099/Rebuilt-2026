package com.team4099.lib.pathfollow

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryParameterizer as TrajectoryParameterizer

fun trajectoryFromPath(
    startVelocity: LinearVelocity,
    path: Path,
    endVelocity: LinearVelocity,
    trajectoryConfig: TrajectoryConfig
): Trajectory {
  if (!path.built) path.build()

  val wpilibStates =
      TrajectoryParameterizer.parameterize(
              path.splinePoints,
              trajectoryConfig.constraints,
              startVelocity.inMetersPerSecond,
              endVelocity.inMetersPerSecond,
              trajectoryConfig.maxLinearVelocity.inMetersPerSecond,
              trajectoryConfig.maxLinearAcceleration.inMetersPerSecondPerSecond,
              false)
          .samples

  val states =
      wpilibStates.mapIndexed { index, state ->
        var headingTarget =
            if (index == 0) {
              path.startingPose.rotation
            } else if (index == wpilibStates.size - 1) {
              path.endingPose.rotation
            } else {
              val tailMap = path.headingPointMap.tailMap(index)
              if (tailMap.size == 0) {
                path.endingPose.rotation
              } else {
                path.headingPointMap[tailMap.firstKey()]
              }
            }

        if (headingTarget == null) {
          headingTarget = path.endingPose.rotation
        }
        /*
         #TO DO

         Look at wpilib to see if we can generate a trapezoidal profile
          using max angular vel and acceleration

          Look back to this if rotation is jittery
        */
        TrajectoryState(
            state.time.seconds,
            Pose2d(Translation2d(state.pose.translation), headingTarget),
            state.pose.rotation.angle,
            state.forwardVelocity().meters.perSecond,
            state.forwardAcceleration().meters.perSecond.perSecond)
      }

  return Trajectory(states)
}
