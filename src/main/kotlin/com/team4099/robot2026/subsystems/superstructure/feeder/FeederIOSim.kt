package com.team4099.robot2026.subsystems.superstructure.feeder

import com.team4099.lib.math.clamp
import com.team4099.robot2025.subsystems.Feeders.FeederIO
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FeederConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object FeederIOSim : FeederIO {
  private var appliedVoltage = 0.0.volts
  private val subsystemSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX44(1),
              FeederConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1 / FeederConstants.GEAR_RATIO),
          DCMotor.getKrakenX44(1))

  override fun updateInputs(inputs: FeederIO.FeederIOInputs) {
    subsystemSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.feederVelocity = subsystemSim.angularVelocityRPM.rotations.perMinute
    inputs.feederAppliedVoltage = appliedVoltage
    inputs.feederSupplyCurrent = subsystemSim.currentDrawAmps.amps
    inputs.feederStatorCurrent = 0.0.amps
    inputs.feederTemp = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -FeederConstants.VOLTAGE_COMPENSATION, FeederConstants.VOLTAGE_COMPENSATION)
    subsystemSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }
}
