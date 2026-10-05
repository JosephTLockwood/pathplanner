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
- Paths and autos must use the `2025.0` file format (the GUI's classic path and auto editors).
  Files from the new path editor (`2027.1` format) can't be loaded by this build yet.

## Versions

| Version | Built from |
| --- | --- |
| `2027.0.0-alpha-7-commandsv3-1` | `new-path-2027-commands-v3` @ `8f7af5fb` |
