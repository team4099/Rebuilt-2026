package com.team4099.robot2026.config

import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.Consumer
import org.team4099.lib.joystick.XboxOneGamepad

object ControlBoard {

  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val driverRumbleConsumer =
      Consumer<Boolean> {
        driver.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
      }

  val operatorRumbleConsumer =
      Consumer<Boolean> {
        operator.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
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
  val unjam = Trigger { operator.bButton && operator.dPadUp }

  // val climb = Trigger { driver.aButton }
  val forceIdle = Trigger { driver.dPadDown || operator.dPadDown }
  val leftTrenchOTF = Trigger { driver.leftShoulderButton }
  val rightTrenchOTF = Trigger { driver.rightShoulderButton }

  val climbOTF = Trigger { driver.yButton }

  val eject = Trigger { driver.dPadUp }

  val prepScore = Trigger { operator.yButton }
  val forceIntakeFullUp = Trigger { operator.rightTriggerAxis > .5 }
  val rotateTrench = Trigger { operator.rightShoulderButton }
  val forceIntakeFullDown = Trigger { operator.leftTriggerAxis > .5 }
  val rotateBump = Trigger { operator.leftShoulderButton }
  val jiggle = Trigger { operator.aButton }

  // val prepClimb = Trigger { operator.aButton }

  val forceHome = Trigger { operator.startButton && operator.selectButton }
}
