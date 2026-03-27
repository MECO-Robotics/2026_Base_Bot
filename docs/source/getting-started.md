# Getting Started

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
4. Build the robot code:

```powershell
./gradlew build
```

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
