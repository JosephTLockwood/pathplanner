package com.pathplanner.lib.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

/**
 * A description of a command that is independent of the command-based framework being used.
 *
 * <p>PathPlanner autos and event markers describe their commands using command specs. Each
 * supported command framework then builds its own command objects from these descriptions:
 *
 * <ul>
 *   <li>Commands v2: {@link com.pathplanner.lib.auto.CommandUtil#buildCommand(CommandSpec,
 *       boolean)}
 *   <li>Commands v3: {@link com.pathplanner.lib.command3.CommandUtil#buildCommand(CommandSpec,
 *       boolean)}
 * </ul>
 *
 * <p>This allows paths and autos to be loaded without depending on a specific version of the
 * command framework.
 */
public sealed interface CommandSpec {
  /** A command that does nothing. */
  record None() implements CommandSpec {}

  /**
   * A command that waits for a given amount of time
   *
   * @param waitTimeSeconds The time to wait, in seconds
   */
  record Wait(double waitTimeSeconds) implements CommandSpec {}

  /**
   * A command that was registered with NamedCommands
   *
   * @param name The name the command was registered with
   */
  record Named(String name) implements CommandSpec {}

  /**
   * A command that follows a path
   *
   * @param pathName The name of the path file, or the name of the Choreo trajectory
   * @param choreoPath True if this path is a Choreo trajectory
   */
  record FollowPath(String pathName, boolean choreoPath) implements CommandSpec {
    /**
     * Load the path that this command should follow
     *
     * @param mirror Should the path be mirrored to the opposite side of the field
     * @return The path to follow
     * @throws IOException if the file cannot be read
     * @throws ParseException If the JSON cannot be parsed
     */
    public PathPlannerPath loadPath(boolean mirror) throws IOException, ParseException {
      PathPlannerPath path =
          choreoPath
              ? PathPlannerPath.fromChoreoTrajectory(pathName)
              : PathPlannerPath.fromPathFile(pathName);
      return mirror ? path.mirrorPath() : path;
    }
  }

  /** How the commands in a group are run */
  enum GroupType {
    /** Run the commands one after another */
    SEQUENTIAL,
    /** Run the commands at the same time, finishing when all of them have finished */
    PARALLEL,
    /** Run the commands at the same time, finishing when any of them has finished */
    RACE,
    /** Run the commands at the same time, finishing when the first command has finished */
    DEADLINE
  }

  /**
   * A group of commands
   *
   * @param type How the commands in the group are run
   * @param commands The commands in the group. For a deadline group, the first command is the
   *     deadline.
   */
  record Group(GroupType type, List<CommandSpec> commands) implements CommandSpec {}

  /**
   * An already constructed command object. The command must be a command from the command framework
   * that is being used (an {@code org.wpilib.command2.Command} for Commands v2, or an {@code
   * org.wpilib.command3.Command} for Commands v3).
   *
   * @param command The command object
   */
  record Prebuilt(Object command) implements CommandSpec {}

  /**
   * Create a spec that runs an already constructed command object
   *
   * @param command The command object. This must be a command from the command framework that is
   *     being used (an {@code org.wpilib.command2.Command} for Commands v2, or an {@code
   *     org.wpilib.command3.Command} for Commands v3).
   * @return Command spec for the given command
   */
  static CommandSpec of(Object command) {
    return new Prebuilt(command);
  }

  /**
   * Create a command spec from json
   *
   * @param commandJson {@link org.json.simple.JSONObject} representing a command
   * @param choreoPaths Should path commands load Choreo trajectories
   * @return The command spec defined by the given json object
   */
  static CommandSpec fromJson(JSONObject commandJson, boolean choreoPaths) {
    String type = (String) commandJson.get("type");
    JSONObject data = (JSONObject) commandJson.get("data");

    return switch (type) {
      case "wait" -> new Wait(waitTimeFromJson(data));
      case "named" -> new Named((String) data.get("name"));
      case "path" -> new FollowPath((String) data.get("pathName"), choreoPaths);
      case "sequential", "parallel", "race", "deadline" ->
          new Group(
              GroupType.valueOf(type.toUpperCase(Locale.ROOT)),
              childrenFromJson(data, choreoPaths));
      default -> new None();
    };
  }

  /**
   * Get all of the path commands in this command, in the order they appear
   *
   * @return List of path commands
   */
  default List<FollowPath> getPathCommands() {
    List<FollowPath> paths = new ArrayList<>();
    switch (this) {
      case FollowPath path -> paths.add(path);
      case Group group -> group.commands().forEach(c -> paths.addAll(c.getPathCommands()));
      default -> {}
    }
    return paths;
  }

  private static double waitTimeFromJson(JSONObject dataJson) {
    Object waitTime = dataJson.get("waitTime");
    if (waitTime instanceof Number number) {
      return number.doubleValue();
    }

    // Choreo expressions store their value in a separate field
    return ((Number) ((JSONObject) waitTime).get("val")).doubleValue();
  }

  private static List<CommandSpec> childrenFromJson(JSONObject dataJson, boolean choreoPaths) {
    List<CommandSpec> commands = new ArrayList<>();
    for (var cmdJson : (JSONArray) dataJson.get("commands")) {
      commands.add(fromJson((JSONObject) cmdJson, choreoPaths));
    }
    return commands;
  }
}
