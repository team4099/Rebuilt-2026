package com.team4099.robot2026.auto

import bline.commands.FollowPathV2
import bline.path.Path
import java.util.function.BooleanSupplier

/** Applies the selected field side and odometry reset to each autonomous path. */
class AutoPathBuilder(
    private val builder: FollowPathV2.Builder,
    private val shouldMirror: BooleanSupplier,
) {
  fun build(path: Path, resetPose: Boolean = true): FollowPathV2 {
    val command = builder.build(path).withShouldMirror(shouldMirror)
    return if (resetPose) command.withPoseReset() else command
  }
}
