package com.pathplanner.lib.auto;

import com.pathplanner.lib.util.PPLibTesting;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * Utility class for managing named conditions. A condition branch in a 2027 path or auto is taken
 * when the condition registered with its name is true.
 *
 * <p>Conditions are independent of the command framework, so the same conditions are used by the
 * Commands v2 and Commands v3 versions of PathPlannerLib.
 */
public class NamedConditions {
  private static final HashMap<String, BooleanSupplier> namedConditions = new HashMap<>();

  static {
    PPLibTesting.addResetHook(NamedConditions::clearAll);
  }

  private NamedConditions() {}

  /**
   * Registers a condition with the given name.
   *
   * @param name the name of the condition. This must match the condition name in the app exactly.
   * @param condition the condition to register
   */
  public static void registerCondition(String name, BooleanSupplier condition) {
    namedConditions.put(name, condition);
  }

  /**
   * Registers a map of conditions with their associated names.
   *
   * @param conditions the map of conditions to register
   */
  public static void registerConditions(Map<String, BooleanSupplier> conditions) {
    namedConditions.putAll(conditions);
  }

  /**
   * Returns whether a condition with the given name has been registered.
   *
   * @param name the name of the condition to check
   * @return true if a condition with the given name has been registered, false otherwise
   */
  public static boolean hasCondition(String name) {
    return namedConditions.containsKey(name);
  }

  /**
   * Returns the condition registered with the given name.
   *
   * @param name the name of the condition
   * @return the condition registered with the given name, or a condition that is always false if no
   *     condition has been registered with that name
   */
  public static BooleanSupplier getCondition(String name) {
    if (name != null && hasCondition(name)) {
      return namedConditions.get(name);
    }

    DriverStationErrors.reportWarning(
        "PathPlanner attempted to use a condition '"
            + name
            + "' that has not been registered with NamedConditions.registerCondition",
        false);
    return () -> false;
  }

  /** Clear all registered conditions */
  public static void clearAll() {
    namedConditions.clear();
  }
}
