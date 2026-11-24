package org.team4099.lib.units.derived

import org.team4099.lib.units.Value
import org.team4099.lib.units.base.Mass

/** Attaches square metres to a mass expressed in kilograms. */
val Mass.meterSquared: MomentOfInertia
  get() = Value(value)
