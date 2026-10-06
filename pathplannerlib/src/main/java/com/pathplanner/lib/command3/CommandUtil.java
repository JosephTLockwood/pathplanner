package com.pathplanner.lib.command3;

import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.CommandSpec;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
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
      // Empty parallel groups would never finish
      case CommandSpec.Group group when group.commands().isEmpty() -> none();
      case CommandSpec.Group group -> {
        Command[] commands = buildCommands(group.commands(), mirror);
        yield switch (group.type()) {
          case SEQUENTIAL -> Command.sequence(commands).withAutomaticName();
          case PARALLEL -> Command.parallel(commands).withAutomaticName();
          case RACE -> Command.race(commands).withAutomaticName();
          // The deadline is the only required command, the others are canceled when it finishes
          case DEADLINE ->
              new ParallelGroupBuilder()
                  .requiring(commands[0])
                  .optional(Arrays.copyOfRange(commands, 1, commands.length))
                  .withAutomaticName();
        };
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
