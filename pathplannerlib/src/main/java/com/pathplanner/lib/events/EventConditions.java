package com.pathplanner.lib.events;

import java.util.HashMap;
import java.util.Map;

/**
 * The current state of all event and point towards zone conditions. These are updated by the events
 * handled while following a path, and are read by the event triggers of each command framework.
 *
 * <p>This is used internally, and should not need to be used by teams.
 */
public final class EventConditions {
  private static final Map<String, Boolean> eventConditions = new HashMap<>();
  private static final Map<String, Long> eventPulses = new HashMap<>();
  private static final Map<String, Boolean> zoneConditions = new HashMap<>();

  private EventConditions() {}

  /**
   * Get the value of an event condition
   *
   * @param name The name of the event
   * @return True if the event is currently active
   */
  public static boolean isEventActive(String name) {
    return eventConditions.getOrDefault(name, false);
  }

  /**
   * Set the value of an event condition
   *
   * @param name The name of the event
   * @param active The value of the condition
   */
  public static void setEventActive(String name, boolean active) {
    eventConditions.put(name, active);
  }

  /**
   * Get the number of times a one-shot event has been fired. Triggers can compare this against the
   * last count they saw to detect that the event fired since they were last polled.
   *
   * @param name The name of the event
   * @return The number of times the event has been fired
   */
  public static long getEventPulseCount(String name) {
    return eventPulses.getOrDefault(name, 0L);
  }

  /**
   * Fire a one-shot event
   *
   * @param name The name of the event
   */
  public static void pulseEvent(String name) {
    eventPulses.merge(name, 1L, Long::sum);
  }

  /**
   * Get the value of a point towards zone condition
   *
   * @param name The name of the point towards zone
   * @return True if the robot is within the zone
   */
  public static boolean isWithinZone(String name) {
    return zoneConditions.getOrDefault(name, false);
  }

  /**
   * Set the value of a point towards zone condition
   *
   * @param name The name of the point towards zone
   * @param withinZone Is the robot within the zone
   */
  public static void setWithinZone(String name, boolean withinZone) {
    zoneConditions.put(name, withinZone);
  }

  /** Reset all conditions. This should only be used for testing. */
  public static void reset() {
    eventConditions.clear();
    eventPulses.clear();
    zoneConditions.clear();
  }
}
