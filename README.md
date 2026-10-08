# PathplannerLib vendordep (WPILib 2027 alpha + Commands v3)

This branch only hosts a vendordep and Maven repository for the PathplannerLib build on the
[`new-path-2027-commands-v3`](https://github.com/JosephTLockwood/pathplanner/tree/new-path-2027-commands-v3)
branch. It is meant for robot code using **WPILib 2027.0.0-alpha-7**.

## Install

In VS Code, run **WPILib: Manage Vendor Libraries** → **Install new library (online)** and paste:

```
https://raw.githubusercontent.com/JosephTLockwood/pathplanner/vendordep/PathplannerLib.json
```

Or download that file into your project's `vendordeps/` folder.

It uses the same UUID as the official PathplannerLib vendordep, so it replaces it. Remove any other
`PathplannerLib.json` from `vendordeps/` first.

## Using it

- **Java only.** Commands v3 is Java-only, so no C++ library is published here.
- Use it with either the `CommandsV3.json` or the `CommandsV2.json` vendordep (WPILib won't allow
  both in one project).
  - Commands v3: import from `com.pathplanner.lib.command3` (`AutoBuilder`, `NamedCommands`,
    `EventTrigger`, `PathPlannerAuto`, ...). `AutoBuilder.configure(...)` takes your drive
    `Mechanism`s as the last argument.
  - Commands v2: the usual `com.pathplanner.lib.auto` / `com.pathplanner.lib.commands` API.
- From `-2`, paths and autos saved by the new editor (`2027.1` format) load and run, and `2025.0`
  files still do. `PathPlannerAuto` and `AutoBuilder.buildAutoChooser` take either. Load a
  `2027.1` path with `PathGraph.fromPathFile` and follow it with `AutoBuilder.followPath`.
  - Every pose the library takes or returns is in WPILib's blue alliance origin, as with `2025.0`
    files. The new editor saves poses from the center of the field, and the library converts them
    when a file is loaded, using `FlippingUtil.fieldSizeX`/`fieldSizeY`. Loading fails if those
    disagree with the field size in `navgrid.json`.
  - Condition branches read `NamedConditions.registerCondition(name, condition)`. External command
    steps read `NamedCommands`.
  - Commands v3 only. Commands v2 still runs `2025.0` files only.
- `-1` reads `2025.0` files only.

## Versions

| Version | Built from |
| --- | --- |
| `2027.0.0-alpha-7-commandsv3-2` | `path2-2027-1` @ `7c125f72` |
| `2027.0.0-alpha-7-commandsv3-1` | `new-path-2027-commands-v3` @ `8f7af5fb` |
