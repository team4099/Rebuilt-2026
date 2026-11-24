# PathPlanner 2027 sources

Pinned from the official [PathPlanner 2027 branch](https://github.com/mjansen4857/pathplanner/tree/7fbd7980c5f81874d56a5b64c39fb8d8c3037352), commit `7fbd7980c5f81874d56a5b64c39fb8d8c3037352`. Local changes in `SwerveSetpointGenerator.java` retain the return values from WPILib 2027’s immutable optimization and desaturation methods. Other Java sources are unchanged.

FalconUtils 2.0.0-beta.2 bundles an older PathPlanner build referencing WPILib classes removed in Alpha 8. The root build extracts only the team library from FalconUtils and compiles these compatible PathPlanner sources. Replace this source directory and its build entries with an official 2027 vendordep when available. Upstream license notices are retained.
