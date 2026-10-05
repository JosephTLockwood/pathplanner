package com.pathplanner.lib.command3;

import com.pathplanner.lib.util.PPLibTesting;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Pair;

/** Utility class for managing named Commands v3 commands */
public class NamedCommands {
  private static final HashMap<String, Command> namedCommands = new HashMap<>();

  static {
    PPLibTesting.addResetHook(NamedCommands::clearAll);
  }

  private NamedCommands() {}

  /**
   * Registers a command with the given name.
   *
   * @param name the name of the command
   * @param command the command to register
   */
  public static void registerCommand(String name, Command command) {
    namedCommands.put(name, command);
  }

  /**
   * Registers a list of commands with their associated names.
   *
   * @param commands the list of commands to register
   */
  public static void registerCommands(List<Pair<String, Command>> commands) {
    for (var pair : commands) {
      registerCommand(pair.getFirst(), pair.getSecond());
    }
  }

  /**
   * Registers a map of commands with their associated names.
   *
   * @param commands the map of commands to register
   */
  public static void registerCommands(Map<String, Command> commands) {
    namedCommands.putAll(commands);
  }

  /**
   * Returns whether a command with the given name has been registered.
   *
   * @param name the name of the command to check
   * @return true if a command with the given name has been registered, false otherwise
   */
  public static boolean hasCommand(String name) {
    return namedCommands.containsKey(name);
  }

  /**
   * Returns the command registered with the given name. Unlike Commands v2, Commands v3 commands
   * can be used in multiple compositions, so the registered command is returned directly.
   *
   * @param name the name of the command
   * @return the command registered with the given name, or a command that does nothing if no
   *     command has been registered with that name
   */
  public static Command getCommand(String name) {
    if (hasCommand(name)) {
      return namedCommands.get(name);
    } else {
      DriverStationErrors.reportWarning(
          "PathPlanner attempted to create a command '"
              + name
              + "' that has not been registered with NamedCommands.registerCommand",
          false);
      return CommandUtil.none();
    }
  }

  /** Clear all registered commands */
  public static void clearAll() {
    namedCommands.clear();
  }
}
