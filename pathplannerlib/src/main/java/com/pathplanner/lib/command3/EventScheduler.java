package com.pathplanner.lib.command3;

import com.pathplanner.lib.auto.CommandSpec;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.EventConditions;
import com.pathplanner.lib.events.EventSchedulerBase;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.json.simple.parser.ParseException;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * Handles the events of a trajectory while it is being followed, using Commands v3.
 *
 * <p>Events are handled by a command that should be forked by the command following the trajectory.
 * That command waits for each event's time along the trajectory, then handles it. The commands of
 * event markers are forked as children of the events command, so they will be canceled when the
 * path following command ends.
 *
 * <pre>{@code
 * public void run(Coroutine coroutine) {
 *   PathPlannerTrajectory trajectory = ...;
 *   Timer timer = Timer.createStarted();
 *   coroutine.fork(eventScheduler.eventsCommand(trajectory, timer::get));
 *
 *   // ... follow the trajectory
 * }
 * }</pre>
 */
public class EventScheduler extends EventSchedulerBase {
  private final Map<CommandSpec, Command> eventCommands = new IdentityHashMap<>();
  private Coroutine coroutine;

  /** Create a new EventScheduler */
  public EventScheduler() {}

  /**
   * Build the commands for the event markers of a path ahead of time, so they do not need to be
   * built while the path is being followed.
   *
   * @param path The path to build event commands for
   * @return Set of mechanisms required by the event commands
   */
  public Set<Mechanism> buildEventCommands(PathPlannerPath path) {
    Set<Mechanism> allReqs = new HashSet<>();

    for (EventMarker m : path.getEventMarkers()) {
      if (m.command() != null) {
        allReqs.addAll(getEventCommand(m.command()).requirements());
      }
    }

    return allReqs;
  }

  /**
   * Get the event requirements for the given path
   *
   * @param path The path to get all requirements for
   * @return Set of event requirements for the given path
   */
  public static Set<Mechanism> getSchedulerRequirements(PathPlannerPath path) {
    return new EventScheduler().buildEventCommands(path);
  }

  /**
   * Create a command that handles the events of a trajectory. This command should be forked by the
   * command that is following the trajectory, so that it is canceled when the path ends. It does
   * not require any mechanisms.
   *
   * @param trajectory The trajectory being followed
   * @param trajectoryTime Supplier for the current time along the trajectory, in seconds
   * @return Command that handles the trajectory's events
   */
  public Command eventsCommand(PathPlannerTrajectory trajectory, DoubleSupplier trajectoryTime) {
    Deque<Event> upcomingEvents =
        new ArrayDeque<>(
            trajectory.getEvents().stream()
                .sorted(Comparator.comparingDouble(Event::getTimestampSeconds))
                .toList());

    return Command.noRequirements(
            coroutine -> {
              this.coroutine = coroutine;
              // An event command that can't be scheduled should not cancel the path
              coroutine.setCancelOnForkFailure(false);

              while (!upcomingEvents.isEmpty()) {
                double eventTime = upcomingEvents.peekFirst().getTimestampSeconds();
                coroutine.waitUntil(() -> trajectoryTime.getAsDouble() >= eventTime);
                upcomingEvents.removeFirst().handleEvent(this);
              }

              // Every event has been handled. Keep running until the path ends so that event
              // commands that are still running are not canceled early.
              coroutine.park();
            })
        .whenCanceled(
            () -> {
              this.coroutine = null;

              // Cancel any events that were never reached
              for (Event e : upcomingEvents) {
                e.cancelEvent(this);
              }
              upcomingEvents.clear();
            })
        .named("PathPlanner Events");
  }

  @Override
  protected void scheduleCommand(CommandSpec command) {
    Command eventCommand = getEventCommand(command);

    // Forking starts the command immediately, canceling any other commands with shared
    // requirements
    var result = coroutine.fork(eventCommand);
    if (result.failed()) {
      DriverStationErrors.reportWarning(
          "PathPlanner could not schedule event command '"
              + eventCommand.name()
              + "' because a higher priority command is using its mechanisms",
          false);
    }
  }

  @Override
  protected void cancelCommand(CommandSpec command) {
    Command eventCommand = eventCommands.get(command);
    if (eventCommand != null) {
      coroutine.scheduler().cancel(eventCommand);
    }
  }

  @Override
  protected void handleOneShotTrigger(String eventName) {
    EventConditions.pulseEvent(eventName);
  }

  private Command getEventCommand(CommandSpec spec) {
    return eventCommands.computeIfAbsent(spec, EventScheduler::buildEventCommand);
  }

  private static Command buildEventCommand(CommandSpec spec) {
    try {
      return CommandUtil.buildCommand(spec, false);
    } catch (IOException | ParseException e) {
      DriverStationErrors.reportError(
          "Failed to build event command: " + e.getMessage(), e.getStackTrace());
      return CommandUtil.none();
    }
  }
}
