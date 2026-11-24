# maple-sim 2027 source port

Pinned from [Grim-Raid/maple-sim](https://github.com/Grim-Raid/maple-sim/tree/b6dc92eda0ac2983838d008c52f85916ee68fea2), commit `b6dc92eda0ac2983838d008c52f85916ee68fea2`, proposed in [upstream PR #163](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/pull/163).

The published maple-sim 0.4.0-beta artifact uses WPILib 2026 packages and cannot run with WPILib 2027. This source port preserves the existing REBUILT field and drivetrain simulation while compiling against the project's WPILib version. The Java sources are unchanged from the pinned commit and use dyn4j 5.0.2. See LICENSE.txt for the upstream license.

Replace this source directory and its `sourceSets`/dyn4j entries in `build.gradle` with the official vendordep when a compatible release is available.
