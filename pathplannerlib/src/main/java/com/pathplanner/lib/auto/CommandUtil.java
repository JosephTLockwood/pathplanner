package com.pathplanner.lib.auto;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.*;

/** Utility class for building Commands v2 commands used in autos */
public class CommandUtil {

  /**
   * Wraps a command with a functional command that calls the command's initialize, execute, end,
   * and isFinished methods. This allows a command in the event map to be reused multiple times in
   * different command groups
   *
   * @param eventCommand the command to wrap
   * @return a functional command that wraps the given command
   */
  public static Command wrappedEventCommand(Command eventCommand) {
    return new FunctionalCommand(
        eventCommand::initialize,
        eventCommand::execute,
        eventCommand::end,
        eventCommand::isFinished,
        eventCommand.getRequirements().toArray(Subsystem[]::new));
  }

  /**
   * Builds a command from the given JSON object.
   *
   * @param commandJson the JSON object to build the command from
   * @param loadChoreoPaths Load path commands using choreo trajectories
   * @param mirror Should the paths be mirrored
   * @return a command built from the JSON object
   * @throws IOException if attempting to load a path file that does not exist or cannot be read
   * @throws ParseException If attempting to load a path with JSON that cannot be parsed
   */
  public static Command commandFromJson(
      JSONObject commandJson, boolean loadChoreoPaths, boolean mirror)
      throws IOException, ParseException {
    return buildCommand(CommandSpec.fromJson(commandJson, loadChoreoPaths), mirror);
  }

  /**
   * Builds a command from the given command spec.
   *
   * @param spec the spec describing the command to build
   * @param mirror Should the paths be mirrored
   * @return a command built from the spec
   * @throws IOException if attempting to load a path file that does not exist or cannot be read
   * @throws ParseException If attempting to load a path with JSON that cannot be parsed
   */
  public static Command buildCommand(CommandSpec spec, boolean mirror)
      throws IOException, ParseException {
    return switch (spec) {
      case CommandSpec.None _ -> Commands.none();
      case CommandSpec.Wait wait -> Commands.waitSeconds(wait.waitTimeSeconds());
      case CommandSpec.Named named -> NamedCommands.getCommand(named.name());
      case CommandSpec.FollowPath path -> AutoBuilder.followPath(path.loadPath(mirror));
      case CommandSpec.Sequential group ->
          new SequentialCommandGroup(buildCommands(group.commands(), mirror));
      case CommandSpec.Parallel group ->
          new ParallelCommandGroup(buildCommands(group.commands(), mirror));
      case CommandSpec.Race group -> new ParallelRaceGroup(buildCommands(group.commands(), mirror));
      case CommandSpec.Deadline group -> deadlineGroup(buildCommands(group.commands(), mirror));
      case CommandSpec.Prebuilt prebuilt -> {
        if (prebuilt.command() instanceof Command command) {
          yield command;
        }
        throw new IllegalArgumentException(
            "Prebuilt command spec does not contain a Commands v2 command: " + prebuilt.command());
      }
    };
  }

  private static Command[] buildCommands(List<CommandSpec> specs, boolean mirror)
      throws IOException, ParseException {
    Command[] commands = new Command[specs.size()];
    for (int i = 0; i < specs.size(); i++) {
      commands[i] = buildCommand(specs.get(i), mirror);
    }
    return commands;
  }

  private static Command deadlineGroup(Command[] commands) {
    if (commands.length == 0) {
      return Commands.none();
    }

    return new ParallelDeadlineGroup(commands[0], Arrays.copyOfRange(commands, 1, commands.length));
  }
}
