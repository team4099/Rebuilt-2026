package com.team4099.robot2026.config

import com.team4099.robot2026.config.constants.Constants
import java.util.function.Consumer
import org.team4099.lib.joystick.XboxOneGamepad
import org.wpilib.command2.button.Trigger
import org.wpilib.driverstation.GenericHID

object ControlBoard {

  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val driverRumbleConsumer =
      Consumer<Boolean> {
        val intensity = if (it) 1.0 else 0.0
        driver.hid.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, intensity)
        driver.hid.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, intensity)
      }

  val operatorRumbleConsumer =
      Consumer<Boolean> {
        val intensity = if (it) 1.0 else 0.0
        operator.hid.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, intensity)
        operator.hid.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, intensity)
      }

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() = driver.rightXAxis

  val slowMode: Boolean
    get() = driver.rightJoystickButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  val intake = Trigger { driver.leftTriggerAxis > .5 }
  val score = Trigger { driver.rightTriggerAxis > .5 }

  val manualScore = Trigger { operator.bButton && operator.dPadLeft }
  val defenseMode = Trigger { operator.bButton && operator.dPadRight }
  val unjam = Trigger { operator.xButton }

  // val climb = Trigger { driver.aButton }
  val forceIdle = Trigger { driver.dPadDown || operator.dPadDown }

  //  val climbOTF = Trigger { driver.yButton }

  val eject = Trigger { driver.dPadUp || operator.dPadUp }

  val prepScore = Trigger { operator.yButton }
  val forceIntakeUpTrigger = Trigger { operator.rightTriggerAxis > .5 }
  val forceIntakeUpShoulder = Trigger { operator.rightShoulderButton }
  val forceIntakeDownTrigger = Trigger { operator.leftTriggerAxis > .5 }
  val forceIntakeDownShoulder = Trigger { operator.leftShoulderButton }
  val jiggle = Trigger { operator.aButton }

  // val prepClimb = Trigger { operator.aButton }

  val forceHome = Trigger { operator.startButton && operator.selectButton }

  val quasiForward = Trigger { driver.xButton }
  val quasiBackward = Trigger { driver.yButton }
  val dynamicForward = Trigger { driver.aButton }
  val dynamicBackward = Trigger { driver.bButton }
}
