# Getting Started

This page is for teams adopting the base bot for the first time. If you only
read one page before editing code, read this one.

## First-week plan

Use this sequence if you are new to the project:

1. Build the code without changing anything.
2. Read how `RobotContainer` assembles the robot.
3. Replace placeholder constants and hardware IDs.
4. Compose the robot behavior you want in `RobotContainer`.
5. Keep the IO-backed structure intact until a real robot requirement forces a change.

That order matters. Teams usually get into trouble by renaming packages and
rewriting structure before they understand what is already reusable.

## Default student workflow

For most team members, the project should feel like a configuration-and-composition
codebase:

1. Configure the robot in `frc.robot.constants`.
2. Instantiate the subsystems you need.
3. Build commands and button bindings in `RobotContainer`.
4. Tune values and iterate.

That covers most day-to-day robot work. You should not need to edit low-level IO
classes or rewrite reusable subsystem logic just to build a season robot.

## What to understand first

Before editing subsystem code, make sure these ideas are clear:

- Commands decide robot behavior.
- Subsystems own mechanism logic.
- IO classes isolate hardware-specific APIs.
- Real, sim, and replay modes swap hardware access without rewriting subsystem behavior.

If those boundaries make sense, the rest of the repository becomes much easier to extend.

## Student-owned files

The files most students should spend time in are:

- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/constants`
- `src/main/deploy`

Those are the main surfaces for:

- hardware IDs and mechanism constants
- command compositions
- controller bindings
- autonomous chooser and named commands
- PathPlanner assets

## Repository layout

The codebase is structured as a normal WPILib Java project:

- `src/main/java/frc/robot`: robot code, commands, subsystems, utilities, and constants.
- `src/main/deploy`: PathPlanner assets and deploy-time files copied to the roboRIO.
- `vendordeps`: vendor JSON definitions for CTRE, REV, PhotonVision, AdvantageKit, and related libraries.
- `sim`: simulation models and configuration assets.
- `docs`: this documentation site.

## Local setup

1. Install the FRC 2026 WPILib toolchain and Java 17.
2. Clone the repository.
3. Open the project in VS Code or your preferred Java IDE.
4. Build the robot code before making changes:

```powershell
./gradlew build
```

5. If the build fails, fix the environment first. Do not start changing robot code until the unmodified project builds on your machine.

## Beginner customization checklist

After the first successful build, the usual beginner changes are:

1. Update hardware IDs and mechanism constants.
2. Confirm which existing subsystems match your robot mechanisms.
3. Remove or disable unused bindings and autos.
4. Add your robot-specific command compositions in `RobotContainer`.
5. Tune before deciding any subsystem rewrite is necessary.

Safe early targets:

- constants under `frc.robot.constants`
- controller bindings in `RobotContainer`
- command compositions in `RobotContainer`
- PathPlanner autos and deploy assets

Changes to postpone until later:

- drive architecture rewrites
- low-level motor-controller rewrites
- collapsing multiple subsystems into one giant subsystem
- removing sim or replay support because it "looks unnecessary"

## Build the documentation

Install the docs dependencies and render the site locally:

```powershell
cd docs
py -m pip install -r requirements.txt
py -m sphinx -b html source build/html
```

Open `docs/build/html/index.html` in a browser after the build completes.

## Runtime modes

The project switches behavior through `Constants.currentMode`:

- `REAL`: hardware IO implementations are active on the roboRIO.
- `SIM`: physics-backed simulation classes are used on desktop.
- `REPLAY`: IO is disabled and log replay implementations are used instead.

This split lets you write subsystem logic once and swap hardware access layers by mode.

## Where to go next

- Read {doc}`architecture` to understand the assembly flow.
- Read {doc}`subsystems` before adding a new mechanism.
- Read {doc}`customizing` when you are ready to make the project team-specific.
