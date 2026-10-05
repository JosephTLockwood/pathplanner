package com.pathplanner.lib.command3;

import com.pathplanner.lib.events.EventConditions;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.event.EventLoop;

/**
 * A Commands v3 trigger that will be controlled by the placement of event markers/zones in a
 * PathPlannerTrajectory.
 *
 * <p>Event triggers are polled by the scheduler like any other trigger, so commands bound to them
 * are scheduled independently of the path following command, and are not canceled when the path
 * ends. To run a command only while the path is running, give the event marker a command in the GUI
 * instead.
 *
 * <p>Event markers without a zone are active for a single scheduler loop each time the marker is
 * reached.
 */
public class EventTrigger extends Trigger {
  /**
   * Create a new EventTrigger that is polled by the default scheduler
   *
   * @param name The name of the event. This will be the name of the event marker in the GUI
   */
  public EventTrigger(String name) {
    this(Scheduler.getDefault(), name);
  }

  /**
   * Create a new EventTrigger that is polled by the given scheduler
   *
   * @param scheduler The scheduler that polls this trigger and runs its bound commands
   * @param name The name of the event. This will be the name of the event marker in the GUI
   */
  public EventTrigger(Scheduler scheduler, String name) {
    super(scheduler, pollCondition(name));
  }

  /**
   * Create a new EventTrigger that is polled by the given event loop
   *
   * @param scheduler The scheduler that runs this trigger's bound commands
   * @param eventLoop The event loop that polls this trigger
   * @param name The name of the event. This will be the name of the event marker in the GUI
   */
  public EventTrigger(Scheduler scheduler, EventLoop eventLoop, String name) {
    super(scheduler, eventLoop, pollCondition(name));
  }

  /**
   * Create a condition that is true while the given event is active. Events without a zone are
   * active for exactly one poll of the condition after they are reached, so each condition should
   * only be polled by a single trigger.
   *
   * @param name The name of the event
   * @return Condition for the event
   */
  static BooleanSupplier pollCondition(String name) {
    return new BooleanSupplier() {
      private long lastPulseCount = EventConditions.getEventPulseCount(name);

      @Override
      public boolean getAsBoolean() {
        long pulseCount = EventConditions.getEventPulseCount(name);
        boolean pulsed = pulseCount != lastPulseCount;
        lastPulseCount = pulseCount;

        return pulsed || EventConditions.isEventActive(name);
      }
    };
  }
}
