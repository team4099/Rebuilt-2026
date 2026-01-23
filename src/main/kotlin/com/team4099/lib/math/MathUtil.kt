package com.team4099.lib.math

import org.team4099.lib.units.UnitKey
import org.team4099.lib.units.Value
import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.Current

fun <T : UnitKey> clamp(input: Value<T>, lowerBound: Value<Ampere>, upperBound: Current): Value<T> {
  return maxOf(lowerBound, minOf(input, upperBound))
}
