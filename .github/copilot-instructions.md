## Purpose
Make quick, actionable recommendations for code changes in this repository. Focus on the WPILib command-based robot architecture, subsystems/commands patterns, simulation vs real IO, and build/deploy conventions.

## Big-picture architecture (what to know first)
- This project uses WPILib (Java) and the Command-based framework. Key entry points:
  - `src/main/java/frc/robot/Main.java` — application bootstrap (do not modify except to change class reference).
  - `src/main/java/frc/robot/Robot.java` — extends `TimedRobot`; runs `CommandScheduler` and contains match/shift tracking and lifecycle hooks.
  - `src/main/java/frc/robot/RobotContainer.java` — central DI: creates subsystems, default commands, trigger bindings, and the autonomous chooser.
- Subsystems live under `src/main/java/frc/robot/subsystems` and commands under `src/main/java/frc/robot/commands` (autos in `commands/Autos`). Follow existing patterns (singletons for some subsystems like `ArmSubsystem.getInstance()`).

## Simulation vs real hardware
- IO abstraction pattern: pairs like `TurretIO` / `TurretIOSim`. Construction uses `RobotBase.isSimulation()` in `RobotContainer` to pick the implementation. When adding hardware, follow this pattern: create an `XxxIO` interface and a `XxxIOSim` for simulation.
- There is a small standalone sim runner for turret work: `src/main/java/frc/robot/sim/RobotTurretSimRunner.java` which reads/writes simple files and NetworkTables entries (`Sim/Turret`). Use it for offline turret experimentation.

## Build, test, and deploy (practical commands)
- Use the Gradle wrapper in repo root. On Windows Powershell use `.\\gradlew.bat <task>`.
  - Build: `.\\gradlew.bat build` (note: `spotlessApply` is configured to run before `build`).
  - Tests: `.\\gradlew.bat test` (the project uses JUnit 5).
  - Deploy to RoboRIO: `.\\gradlew.bat deploy` (GradleRIO `deploy` artifacts are configured in `build.gradle`).
- Desktop simulation is enabled in `build.gradle` (`wpi.sim.addGui().defaultEnabled = true`). Use the WPILib tools/IDE or GradleRIO simulation tasks (check Gradle tasks in your environment) to run simulations.

## Project-specific conventions & patterns
- Auto chooser: `RobotContainer` builds a `SendableChooser` via `AutoBuilder.buildAutoChooser()` and registers autos (see `BasicAutoRed`/`BasicAutoBlue`). Add new autos by adding options to that chooser.
- Default commands: subsystems typically set a default command in `RobotContainer` (e.g. `s_Swerve.setDefaultCommand(new SwerveCom(...))`). Prefer command-based default patterns when adding behavior.
- NetworkTables keys and dashboards: camera stream is configured in `Robot()` using `NetworkTableInstance` with a hardcoded IP for the Limelight — be aware of that (see `Robot` constructor). Elastic dashboard widgets use booleans published by `publishDriverDashboardBooleans()` in `RobotContainer`.
- Named triggers: controller/button mappings are declared centrally in `RobotContainer.configureBindings()` using `Trigger` and command factories (`Commands.runOnce`, `Commands.runEnd`, `.whileTrue()` patterns).

## External integration points
- Vendored dependency files live in `vendordeps/` (several vendor libraries are present). `build.gradle` references a GitHub Maven registry — CI / local runs require `GITHUB_TOKEN`/`GITHUB_ACTOR` when accessing private packages.
- Pathplanner assets: `src/main/deploy/pathplanner` contains path and auto files used by the team; `RobotContainer` uses `com.pathplanner.lib.auto.AutoBuilder`.
- Camera/Limelight stream: `Robot` sets up `CameraPublisher/limelight-driver` table entries with a fixed IP — update carefully if network or camera IP changes.

## Code style and checks
- Spotless + Google Java Format are enforced via `build.gradle`. `build` depends on `spotlessApply`/`spotlessCheck`.
- Java language level: Java 17 (see `build.gradle` `sourceCompatibility` / `targetCompatibility`).

## Safe edit patterns & examples
- To add a simulation-friendly subsystem:
  1. Create `XxxIO` and `XxxIOSim` classes mirroring the `turret` pattern.
  2. In `RobotContainer`, construct with `RobotBase.isSimulation() ? new XxxIOSim() : new XxxIO()`.
  3. Register default commands and expose any dashboard booleans in `publishDriverDashboardBooleans()`.
- To add an autonomous path: add PathPlanner files to `src/main/deploy/pathplanner/paths` and register a new `Auto` command in `commands/Autos`, then add it to the `autoChooser` in `RobotContainer`.

## Where to look for examples
- Entry and lifecycle: `src/main/java/frc/robot/Robot.java` and `Main.java`
- Subsystems & commands: `src/main/java/frc/robot/subsystems` and `src/main/java/frc/robot/commands`
- Turret IO pattern: `src/main/java/frc/robot/subsystems/turret/TurretIO.java` and `TurretIOSim.java` (constructed in `RobotContainer`).
- Simulation runner: `src/main/java/frc/robot/sim/RobotTurretSimRunner.java`

## Units convention
- All measurable units' variable should be using "edu.wpi.first.units.Measure.*"
- All units themselves should be using "edu.wpi.first.units.*"
- Import "edu.wpi.first.units.[specific measureable units]" once, then just use [specific measureable units] later in the code

## If something is unclear
- Ask for which subsystem or workflow you want the agent to change (e.g., "modify turret aiming logic" or "add a new auto path"). Provide the target file(s) or test to update. I can then make focused edits and run the build/tests.

---
If you'd like, I can merge this into an existing `.github/copilot-instructions.md` (none was found) or iterate with more examples (small PRs, tests, or simulated runs). What section should I expand or clarify next?
