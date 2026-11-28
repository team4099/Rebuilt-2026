package com.team4099.robot2026.config.constants

import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.signals.AnimationDirectionValue
import com.ctre.phoenix6.signals.RGBWColor
import org.wpilib.util.Color

object LedConstants {
  const val START_INDEX = 8
  const val END_INDEX = 399

  enum class CandleState(val request: ControlRequest) {
    NOTHING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.GHOST_WHITE))),
    FIRST_WARNING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.YELLOW))),
    SECOND_WARNING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.RED))),
    TEST(
        RainbowAnimation(START_INDEX, END_INDEX)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(20.0)),
    BLUE_DISABLED(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.NAVY))),
    RED_DISABLED(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.DARK_RED))),
    IS_ALIGNING(
        RainbowAnimation(START_INDEX, END_INDEX)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(20.0)),
    SHOOTING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.FUCHSIA))),
    INTAKING_FUEL(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.ORANGE))),
    MANUAL_SCORE(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.RED))),
    UNJAM(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.GRAY))),
  }
}
