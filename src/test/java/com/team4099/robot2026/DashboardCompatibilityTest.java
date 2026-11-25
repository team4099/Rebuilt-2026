package com.team4099.robot2026;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.Tunables;

class DashboardCompatibilityTest {
  @Test
  void existingElasticWaitTopicUpdatesTunable() throws Exception {
    try (var nt = NetworkTableInstance.create()) {
      TunableRegistry.registerBackend("/Pre-match", new NetworkTablesTunableBackend(nt, "/Shuffleboard"));
      var wait = Tunables.addDouble("/Pre-match/Wait Time", 0.0);
      var entry = nt.getEntry("/Shuffleboard/Pre-match/Wait Time");
      assertEquals(0.0, entry.getDouble(-1));
      entry.setDouble(2.5);
      long deadline = System.nanoTime() + 1_000_000_000L;
      while (wait.get() != 2.5 && System.nanoTime() < deadline) {
        TunableRegistry.update();
        Thread.sleep(5);
      }
      assertEquals(2.5, wait.get());
    } finally {
      TunableRegistry.reset();
    }
  }
}
