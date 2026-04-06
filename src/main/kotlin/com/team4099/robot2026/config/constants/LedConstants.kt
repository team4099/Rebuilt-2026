package com.team4099.robot2026.config.constants

import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.signals.AnimationDirectionValue
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj.util.Color

object LedConstants {
  const val START_INDEX = 8
  const val END_INDEX = 399

  enum class CandleState(val request: ControlRequest) {
    NOTHING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kGhostWhite))),
    FIRST_WARNING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kYellow))),
    SECOND_WARNING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kRed))),
    TEST(
        RainbowAnimation(START_INDEX, END_INDEX)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(20.0)),
    BLUE_DISABLED(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kNavy))),
    RED_DISABLED(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kDarkRed))),
    IS_ALIGNING(
        RainbowAnimation(START_INDEX, END_INDEX)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(20.0)),
    SHOOTING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kFuchsia))),
    INTAKING_FUEL(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kOrange))),
    MANUAL_SCORE(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kRed))),
    UNJAM(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kGray))),
  }
}
