package com.pathplanner.lib.follower;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.events.TriggerEvent;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;

/**
 * Information about the path that is currently being followed. This is updated by the path
 * following commands, and is used by the triggers of PathPlannerAuto.
 */
public final class ActivePathState {
  private static String currentPathName = "";
  private static PathPlannerTrajectory currentTrajectory = null;
  private static final Map<String, List<Translation2d>> eventStartPositions = new HashMap<>();
  private static final Map<String, List<Translation2d>> eventEndPositions = new HashMap<>();
  private static final Timer trajTimer = new Timer();

  private ActivePathState() {}

  /**
   * Set the currently running trajectory. This will be used to calculate event timing and positions
   * for the triggers of PathPlannerAuto.
   *
   * @param trajectory The currently running trajectory, or null if no trajectory is running
   */
  public static void setCurrentTrajectory(PathPlannerTrajectory trajectory) {
    currentTrajectory = trajectory;
    eventStartPositions.clear();
    eventEndPositions.clear();
    trajTimer.restart();

    if (trajectory == null) {
      currentPathName = "";
      return;
    }

    for (Event e : trajectory.getEvents()) {
      if (e instanceof OneShotTriggerEvent event) {
        Translation2d pos = trajectory.sample(event.getTimestampSeconds()).pose.getTranslation();
        eventStartPositions.computeIfAbsent(event.getEventName(), k -> new ArrayList<>()).add(pos);
        eventEndPositions.computeIfAbsent(event.getEventName(), k -> new ArrayList<>()).add(pos);
      } else if (e instanceof TriggerEvent event) {
        Translation2d pos = trajectory.sample(event.getTimestampSeconds()).pose.getTranslation();
        var positions = event.getValue() ? eventStartPositions : eventEndPositions;
        positions.computeIfAbsent(event.getEventName(), k -> new ArrayList<>()).add(pos);
      }
    }
  }

  /**
   * Get the currently running trajectory
   *
   * @return The currently running trajectory, or null if no trajectory is running
   */
  public static PathPlannerTrajectory getCurrentTrajectory() {
    return currentTrajectory;
  }

  /**
   * Set the name of the path currently being followed
   *
   * @param pathName The name of the path, or an empty string if no path is being followed
   */
  public static void setCurrentPathName(String pathName) {
    currentPathName = pathName;
  }

  /**
   * Get the name of the path currently being followed
   *
   * @return The name of the path, or an empty string if no path is being followed
   */
  public static String getCurrentPathName() {
    return currentPathName;
  }

  /**
   * Get the field positions where the given event starts along the current trajectory
   *
   * @param eventName The name of the event
   * @return Positions where the event starts
   */
  public static List<Translation2d> getEventStartPositions(String eventName) {
    return eventStartPositions.getOrDefault(eventName, List.of());
  }

  /**
   * Get the field positions where the given event ends along the current trajectory
   *
   * @param eventName The name of the event
   * @return Positions where the event ends
   */
  public static List<Translation2d> getEventEndPositions(String eventName) {
    return eventEndPositions.getOrDefault(eventName, List.of());
  }

  /**
   * Get the time until the given event will next be triggered along the current trajectory
   *
   * @param eventName The name of the event
   * @return Time until the event, in seconds. Empty if the event will not be triggered again.
   */
  public static OptionalDouble getTimeUntilEvent(String eventName) {
    if (currentTrajectory == null) {
      return OptionalDouble.empty();
    }

    double time = trajTimer.get();
    for (Event e : currentTrajectory.getEvents()) {
      if (e.getTimestampSeconds() <= time) {
        continue;
      }

      boolean isEventStart =
          switch (e) {
            case OneShotTriggerEvent event -> eventName.equals(event.getEventName());
            case TriggerEvent event -> event.getValue() && eventName.equals(event.getEventName());
            default -> false;
          };
      if (isEventStart) {
        return OptionalDouble.of(e.getTimestampSeconds() - time);
      }
    }

    return OptionalDouble.empty();
  }
}
