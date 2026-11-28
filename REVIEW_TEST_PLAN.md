# 2027 migration review test plan

- Verify each autonomous mode starts from the selected and correctly mirrored pose; verify subsequent paths do not reset odometry. Check the delay and both field sides on both alliances.
- Exercise trench alignment on both alliances and confirm driver inputs cancel it as expected.
- Verify controller bindings work on first teleop entry and after switching modes without duplicate scheduling; check intake release and scoring together.
- Check the drivetrain simulation responds to velocity and steering commands and compare the behavior with the previous implementation.
- Verify camera log entries are present after the first frame and that replay reconstructs target data correctly for varying target counts.
- Check alerts have distinct IDs and activate and clear for the corresponding faults.
- Confirm characterization produces usable data with the current SysId workflow before running it on hardware.
- Check driver station and dashboard values, log recording, and replay with the 2027 libraries.
