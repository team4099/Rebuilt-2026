package com.team4099.robot2026.subsystems.superstructure.hopper

import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.HopperConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.math.clamp
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object HopperIOSim : HopperIO {
  private var appliedVoltage = 0.0.volts
  private val hopperSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX44Foc(1),
              HopperConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / HopperConstants.GEAR_RATIO,
          ),
          DCMotor.getKrakenX44Foc(1),
      )

  override fun updateInputs(inputs: HopperIO.HopperIOInputs) {
    hopperSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.hopperAngularVelocity = hopperSim.angularVelocityRadPerSec.radians.perSecond
    inputs.hopperAngularAcceleration = hopperSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.hopperAppliedVoltage = appliedVoltage
    inputs.hopperSupplyCurrent = 0.0.amps
    inputs.hopperStatorCurrent = hopperSim.currentDrawAmps.amps
    inputs.hopperTemp = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -HopperConstants.VOLTAGE_COMPENSATION, HopperConstants.VOLTAGE_COMPENSATION)

    hopperSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }
}
