package com.team4099.robot2025.commands.characterization

import com.team4099.robot2025.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.LinkedList

class FeedforwardCharacterizationCommand(private val drive: Drive) : Command() {
  private val velocitySamples: MutableList<Double?> = LinkedList<Double?>()
  private val voltageSamples: MutableList<Double?> = LinkedList<Double?>()
  private val timer = Timer()

  private var delayComplete = false

  init {
    addRequirements(drive)
  }

  override fun initialize() {
    velocitySamples.clear()
    voltageSamples.clear()

    timer.reset()
    timer.start()
    delayComplete = false
  }

  override fun execute() {
    // Initial delay to allow modules to orient
    if (!delayComplete) {
      drive.runCharacterization(0.0)

      if (timer.get() >= FF_START_DELAY) {
        timer.restart() // Start ramp timer
        delayComplete = true
      }
      return
    }

    val voltage = timer.get() * FF_RAMP_RATE
    drive.runCharacterization(voltage)

    velocitySamples.add(drive.ffCharacterizationVelocity)
    voltageSamples.add(voltage)
  }

  override fun end(interrupted: Boolean) {
    drive.runCharacterization(0.0)

    val n = velocitySamples.size
    if (n < 2) {
      println("FF characterization failed: not enough data")
      return
    }

    var sumX = 0.0
    var sumY = 0.0
    var sumXY = 0.0
    var sumX2 = 0.0

    for (i in 0 until n) {
      val x: Double = velocitySamples.get(i)!!
      val y: Double = voltageSamples.get(i)!!

      sumX += x
      sumY += y
      sumXY += x * y
      sumX2 += x * x
    }

    val denominator = n * sumX2 - sumX * sumX
    if (denominator == 0.0) {
      println("FF characterization failed: divide by zero")
      return
    }

    val kS = (sumY * sumX2 - sumX * sumXY) / denominator
    val kV = (n * sumXY - sumX * sumY) / denominator

    val formatter: NumberFormat = DecimalFormat("#0.00000")
    println("********** Drive FF Characterization Results **********")
    println("\tkS: " + formatter.format(kS))
    println("\tkV: " + formatter.format(kV))
  }

  override fun isFinished(): Boolean {
    return false
  }

  companion object {
    private const val FF_RAMP_RATE = 0.1
    private const val FF_START_DELAY = 1.0
  }
}
