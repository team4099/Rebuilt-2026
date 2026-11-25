package com.team4099.robot2026.config

import com.team4099.robot2026.config.constants.Constants
import java.util.function.Consumer
import org.wpilib.driverstation.XboxController
import org.wpilib.command2.button.Trigger
import org.wpilib.driverstation.GenericHID

object ControlBoard {

  private val driver = XboxController(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxController(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxController(Constants.Joysticks.TECHNICIAN_PORT)

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
    get() = -driver.leftX

  val forward: Double
    get() = -driver.leftY

  val turn: Double
    get() = driver.rightX

  val slowMode: Boolean
    get() = driver.rightStickButton

  val resetGyro = Trigger { driver.menuButton && driver.viewButton }

  val intake = Trigger { driver.leftTrigger > .5 }
  val score = Trigger { driver.rightTrigger > .5 }

  val manualScore = Trigger { operator.bButton && operator.dpadLeftButton }
  val defenseMode = Trigger { operator.bButton && operator.dpadRightButton }
  val unjam = Trigger { operator.xButton }

  // val climb = Trigger { driver.aButton }
  val forceIdle = Trigger { driver.dpadDownButton || operator.dpadDownButton }

  //  val climbOTF = Trigger { driver.yButton }

  val eject = Trigger { driver.dpadUpButton || operator.dpadUpButton }

  val prepScore = Trigger { operator.yButton }
  val forceIntakeUpTrigger = Trigger { operator.rightTrigger > .5 }
  val forceIntakeUpShoulder = Trigger { operator.rightBumperButton }
  val forceIntakeDownTrigger = Trigger { operator.leftTrigger > .5 }
  val forceIntakeDownShoulder = Trigger { operator.leftBumperButton }
  val jiggle = Trigger { operator.aButton }

  // val prepClimb = Trigger { operator.aButton }

  val forceHome = Trigger { operator.menuButton && operator.viewButton }

  val quasiForward = Trigger { driver.xButton }
  val quasiBackward = Trigger { driver.yButton }
  val dynamicForward = Trigger { driver.aButton }
  val dynamicBackward = Trigger { driver.bButton }
}
