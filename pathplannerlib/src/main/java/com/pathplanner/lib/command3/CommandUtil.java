package com.pathplanner.lib.command3;

import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.CommandSpec;
import java.io.IOException;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;
import org.wpilib.command3.Command;
import org.wpilib.command3.ParallelGroupBuilder;

/** Utility class for building Commands v3 commands used in autos */
public class CommandUtil {
  private CommandUtil() {}

  /**
   * Create a command that does nothing and finishes immediately
   *
   * @return A command that does nothing
   */
  public static Command none() {
    return Command.noRequirements(_ -> {}).named("None");
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
   * Builds a command from the given command spec. Groups are built with the Commands v3 composition
   * builders, so they require every mechanism required by their commands.
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
      case CommandSpec.None _ -> none();
      case CommandSpec.Wait wait ->
          Command.waitFor(Seconds.of(wait.waitTimeSeconds()))
              .named("Wait " + wait.waitTimeSeconds() + "s");
      case CommandSpec.Named named -> NamedCommands.getCommand(named.name());
      case CommandSpec.FollowPath path -> AutoBuilder.followPath(path.loadPath(mirror));
      case CommandSpec.Sequential group when group.commands().isEmpty() -> none();
      case CommandSpec.Sequential group ->
          Command.sequence(buildCommands(group.commands(), mirror)).withAutomaticName();
      case CommandSpec.Parallel group when group.commands().isEmpty() -> none();
      case CommandSpec.Parallel group ->
          Command.parallel(buildCommands(group.commands(), mirror)).withAutomaticName();
      case CommandSpec.Race group when group.commands().isEmpty() -> none();
      case CommandSpec.Race group ->
          Command.race(buildCommands(group.commands(), mirror)).withAutomaticName();
      case CommandSpec.Deadline group when group.commands().isEmpty() -> none();
      case CommandSpec.Deadline group -> {
        Command[] commands = buildCommands(group.commands(), mirror);
        Command[] others = new Command[commands.length - 1];
        System.arraycopy(commands, 1, others, 0, others.length);

        // The deadline is the only required command, every other command is canceled when the
        // deadline finishes
        yield new ParallelGroupBuilder()
            .requiring(commands[0])
            .optional(others)
            .withAutomaticName();
      }
      case CommandSpec.Prebuilt prebuilt -> {
        if (prebuilt.command() instanceof Command command) {
          yield command;
        }
        throw new IllegalArgumentException(
            "Prebuilt command spec does not contain a Commands v3 command: " + prebuilt.command());
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
}
