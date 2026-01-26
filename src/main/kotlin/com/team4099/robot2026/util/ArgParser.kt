package com.team4099.robot2026.util

/**
 * Parse command-line arguments from the `build.gradle` P-args into their proper type. Make sure
 * that any types put in here match their type definitions in code as well as the arg-specs in the
 * `build.gradle`.
 *
 * @author Nathan Arega
 */
object ArgParser {
  @JvmStatic
  inline fun argToBool(arg: String?): Boolean {
    return when (arg?.lowercase()) {
      "true",
      "t" -> true
      "false",
      "f" -> false
      else ->
          throw IllegalArgumentException(
              "Console-line argument $arg does not match expected type Boolean")
    }
  }

  @JvmStatic
  inline fun <reified T : Enum<T>> argToEnum(arg: String): T {
    val parsed =
        enumValues<T>().firstOrNull { it.name.equals(arg, ignoreCase = true) }
            ?: throw IllegalArgumentException(
                "Console-line argument $arg does not match expected type ${T::class.simpleName}")
    return parsed
  }
}
