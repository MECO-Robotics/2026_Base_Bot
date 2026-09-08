# 16th Note simulation model

Select **16th Note** as the robot model in AdvantageScope's 3D Field tab.
This folder is an AdvantageScope custom robot asset. Copy it into the folder
opened by App > Show Assets Folder, or select the repository's `sim` folder
with App > Use Custom Assets Folder.

The supplied GLB is preserved unchanged. Configuration rotates its Y-up
coordinates to Z-up, centers the square chassis, and places the lowest point
at field height zero. Front-facing heading has not been verified against wiring.
Dimensions: 0.45185 m square, 0.83475 m tall.

Source: `/home/brian/Documents/Codex/2026-09-07/cre/outputs/16th-Note/16th-Note.glb`.
This is a rigid visual model; it does not add drivetrain physics or mechanism
animation. Connect AdvantageScope to the running simulation and assign its
robot pose topic to a Robot object in the 3D Field tab to visualize movement.

Format: https://docs.advantagescope.org/more-features/custom-assets/
