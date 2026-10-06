# Commands v3

WPILib 2027 adds a new command framework, Commands v3 (`org.wpilib.command3`), where commands are written as
coroutines. A robot project can only use one of the Commands v2 or Commands v3 vendordeps, so PathPlannerLib provides
an implementation of its command-based API for each framework.

> **Note**
>
> Commands v3 is only available in Java. C++ and Python robot projects should continue to use the Commands v2 API.
>
{style="note"}

## Which classes to use

Paths, trajectories, controllers, `RobotConfig`, and the rest of PathPlannerLib are shared between both command
frameworks. Only the classes that create or run commands are different. If your project uses the Commands v3 vendordep,
import these classes from the `com.pathplanner.lib.command3` package instead of the Commands v2 versions:

| Commands v2                                        | Commands v3                                          |
|----------------------------------------------------|------------------------------------------------------|
| `com.pathplanner.lib.auto.AutoBuilder`             | `com.pathplanner.lib.command3.AutoBuilder`           |
| `com.pathplanner.lib.auto.NamedCommands`           | `com.pathplanner.lib.command3.NamedCommands`         |
| `com.pathplanner.lib.commands.FollowPathCommand`   | `com.pathplanner.lib.command3.FollowPathCommand`     |
| `com.pathplanner.lib.commands.PathfindingCommand`  | `com.pathplanner.lib.command3.PathfindingCommand`    |
| `com.pathplanner.lib.commands.PathfindThenFollowPath` | `com.pathplanner.lib.command3.PathfindThenFollowPath` |
| `com.pathplanner.lib.commands.PathPlannerAuto`     | `com.pathplanner.lib.command3.PathPlannerAuto`       |
| `com.pathplanner.lib.events.EventTrigger`          | `com.pathplanner.lib.command3.EventTrigger`          |
| `com.pathplanner.lib.events.PointTowardsZoneTrigger` | `com.pathplanner.lib.command3.PointTowardsZoneTrigger` |

## Configure AutoBuilder

Configure AutoBuilder as shown in [Build an Auto](pplib-Build-an-Auto.md), with two differences: import
`com.pathplanner.lib.command3.AutoBuilder`, and pass your drive `Mechanism` (instead of a `Subsystem`) as the last
argument. Path following commands will require that mechanism.

## Named Commands

Named commands are registered with `com.pathplanner.lib.command3.NamedCommands`. Unlike Commands v2, Commands v3
commands can be used in more than one composition, so the registered command is used directly.

```Java
NamedCommands.registerCommand("intake", intake.run(coroutine -> {
  intake.setRollers(1.0);
  coroutine.park();
}).named("Intake"));
```

## Following paths in coroutines

All PathPlannerLib commands can be used inside your own coroutines, the same as any other Commands v3 command.

```Java
Command scoreSequence(PathPlannerPath pathToReef) {
  return Command.noRequirements(coroutine -> {
    coroutine.await(AutoBuilder.followPath(pathToReef));
    coroutine.await(elevator.raise());
    coroutine.await(AutoBuilder.pathfindToPose(loadingStationPose, constraints));
  }).named("Score Sequence");
}
```

## Event markers and triggers

The commands of event markers are forked as children of the path following command. They start when their marker is
reached, are canceled at the end of their zone if the marker is zoned, and are always canceled when the path ends.
Event marker commands may not require the drive mechanism.

`EventTrigger` and `PointTowardsZoneTrigger` are Commands v3 triggers that are polled by the scheduler, so commands
bound to them are scheduled independently of the path following command, and will keep running after the path ends.
Event markers without a zone activate their trigger for a single scheduler loop.

```Java
new EventTrigger("shoot").onTrue(shooter.shoot());
new EventTrigger("intake zone").whileTrue(intake.runRepeatedly(() -> intake.setRollers(1.0)).named("Intake"));
```

The triggers created by a `PathPlannerAuto`, such as `auto.event("shoot")` or `auto.timeElapsed(2.0)`, are also
polled by the scheduler, but are only active while that auto is running.
