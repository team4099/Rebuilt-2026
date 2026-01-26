package com.team4099.robot2026.subsystems.climb

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.ClimbConstants
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object ClimbIOSim : ClimbIO {
  // Modeling the Climb like an elevator
  private val climbSim: ElevatorSim =
      ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          1.0 / ClimbConstants.GEAR_RATIO,
          ClimbConstants.CLIMB_MASS.inKilograms,
          ClimbConstants.DRUM_DIAMETER.inMeters / 2.0,
          ClimbConstants.DOWNWARDS_EXTENSION_LIMIT.inMeters,
          ClimbConstants.UPWARDS_EXTENSION_LIMIT.inMeters,
          false,
          0.0
      )

  private var lastAppliedVoltage = 0.0.volts

  private val climbPIDController =
      ProfiledPIDController(
          ClimbConstants.PID.SIM_KP,
          ClimbConstants.PID.SIM_KI,
          ClimbConstants.PID.SIM_KD,
          TrapezoidProfile.Constraints(
              ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCELERATION))

  override fun updateInputs(inputs: ClimbIO.ClimbInputs) {
    climbSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.climbPosition = climbSim.positionMeters.meters

    inputs.temperature = 0.0.celsius
    inputs.statorCurrent = climbSim.currentDrawAmps.amps
    inputs.supplyCurrent = 0.0.amps
    inputs.appliedVoltage = lastAppliedVoltage

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climbSim.currentDrawAmps))
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            targetVoltage,
            -ClimbConstants.VOLTAGE_COMPENSATION,
            ClimbConstants.VOLTAGE_COMPENSATION)
    lastAppliedVoltage = clampedVoltage

    climbSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setPosition(position: Length) {
    climbPIDController.setGoal(position)
    val pidOutput = climbPIDController.calculate(climbSim.positionMeters.meters)
    setVoltage(pidOutput)
  }

  override fun zeroEncoder() {
    climbSim.setState(0.0, 0.0)
  }

  override fun configPID(
      kP: ProportionalGain<Meter, Volt>,
      kI: IntegralGain<Meter, Volt>,
      kD: DerivativeGain<Meter, Volt>
  ) {
    climbPIDController.setPID(kP, kI, kD)
  }

  override fun configFF(kS: ElectricalPotential, kG: ElectricalPotential) {}
}
