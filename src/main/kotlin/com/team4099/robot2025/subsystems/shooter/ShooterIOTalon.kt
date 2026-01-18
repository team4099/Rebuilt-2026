package com.team4099.robot2025.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_ACCELERATION
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_VELOCITY
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import edu.wpi.first.units.measure.Angle as WPILibAngle
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ShooterIOTalon : ShooterIO {
    private val leaderTalon: TalonFX = TalonFX(Constants.Shooter.LEADER_MOTOR_ID)
    private val followerTalon: TalonFX = TalonFX(Constants.Shooter.FOLLOWER_MOTOR_ID)
    private val motionMagicControl: MotionMagicVoltage = MotionMAgicVoltage()
    private val configs: TalonFXConfiguration = TalonFXConfiguration(-1337.degrees.InDegrees)
    private val leaderSensor = ctreAngularMechanismSensor(
        leaderTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION
    )
    private var leaderStatorCurrentSignal:StatusSignal<WPILibCurrent>
    private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
    private var leaderTempSignal: StatusSignal<WPILibTemperature>
    private var leaderDutyCycle: StatusSignal<Double>
    private var leaderTorqueSignal: StatusSignal<WPICurrent>
    private var leaderVoltageSignal: StatusSignal<Voltage>
    private var leaderPositionSignal: StatusSignal<Angle>
    private var leaderVelocitySignal: StatusSignal<AngularVelocity>

    private var followerStatorCurrentSignal:StatusSignal<WPILibCurrent>
    private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
    private var followerTempSignal: StatusSignal<WPILibTemperature>
    private var followerDutyCycle: StatusSignal<Double>
    private var followerTorqueSignal: StatusSignal<WPICurrent>
    private var followerVoltageSignal: StatusSignal<Voltage>
    private var followerAccelSignal: StatusSignal<AngularAcceleration>
    private var followerPositionSignal: StatusSignal<WPILibAngle>
    private var followerVelocitySignal: StatusSignal<AngularVelocity>

    private var motionMagicTargetVelocity: StatusSignal<Double>
    private var motionMagicTargetPosition: StatusSignal<Double>

    init {
        leaderTalon.clearStickyFaults()
    }
}