package com.team4099.robot2026

import com.team4099.robot2026.config.constants.FeederConstants
import com.team4099.robot2026.config.constants.HopperConstants
import com.team4099.robot2026.config.constants.ShooterConstants
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class SimulationPidConstantsTest {
  @Test
  fun `simulation integral gains are finite`() {
    System.setProperty("robot", "compbot")
    System.setProperty("tuning", "f")
    assertTrue(FeederConstants.PID.SIM_KI.value.isFinite())
    assertTrue(HopperConstants.PID.SIM_KI.value.isFinite())
    assertTrue(ShooterConstants.PID.SIM_KI.value.isFinite())
  }
}
