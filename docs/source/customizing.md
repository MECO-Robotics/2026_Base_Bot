# Customizing the Base Project

## Forking strategy

Treat this repository as a starting point, not a finished robot application.

Recommended adoption flow:

1. Fork the repository for your team.
2. Rename or replace example hardware constants.
3. Keep reusable base patterns intact until a season requirement forces a change.
4. Build robot-specific behavior in commands and higher-level coordination classes.

## Adding a new mechanism

When adding a subsystem, prefer the same structure used by the existing code:

1. Create an IO interface describing the hardware contract.
2. Add real, sim, and replay implementations where practical.
3. Keep the subsystem focused on control behavior and logging.
4. Add command factories or dedicated commands for operator intent.

This avoids hard-coding motor controller APIs into business logic.

## Extending drive and autonomous behavior

The project already includes PathPlanner and characterization utilities. In practice that means:

- autonomous modes should usually be created in PathPlanner and selected through the chooser
- drive tuning should happen through the existing gain and module-limit hooks
- odometry quality should be improved through better constants and vision integration before rewriting the drive stack

## Documentation workflow

Keep the docs in sync with the code when you:

- add or remove subsystems
- change the expected setup process
- switch vendor libraries or hardware assumptions
- introduce new simulation or logging workflows

To rebuild the site:

```powershell
cd docs
py -m sphinx -b html source build/html
```
