# Contributing to OmindOS Manipulation Community

Thank you for helping improve the community learning edition.

## Good first contributions

- Fix an example that does not run.
- Document a tested Python / OS configuration.
- Add a minimal example for one concept.
- Improve comments or learning documentation.
- Add a simulation-only manipulation exercise.
- Report a reproducible collision, kinematics, grasping, or planning issue.

## Contribution principles

1. Keep examples small and runnable.
2. Separate simulation examples from hardware deployment.
3. Do not remove upstream copyright or license notices.
4. Document new dependencies.
5. For algorithm changes, include a minimal example showing the expected result.
6. Avoid committing credentials, private robot addresses, proprietary datasets, or confidential deployment configuration.

## Pull requests

A useful pull request should state:

- what changed;
- why it changed;
- how to run or reproduce it;
- what environment was tested;
- whether it affects simulation, real hardware, or both.

## Hardware safety

Code that works in simulation is not automatically safe for a physical robot. Hardware-facing contributions should clearly state assumptions and required validation.
