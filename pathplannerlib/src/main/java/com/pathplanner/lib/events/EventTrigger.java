package com.pathplanner.lib.events;

import java.util.function.BooleanSupplier;
import org.wpilib.command2.button.Trigger;
import org.wpilib.event.EventLoop;

/**
 * A trigger that will be controlled by the placement of event markers/zones in a
 * PathPlannerTrajectory
 */
public class EventTrigger extends Trigger {

  /**
   * Create a new EventTrigger. This will run on the EventScheduler's event loop, which will be
   * polled any time a path following command is running.
   *
   * @param name The name of the event. This will be the name of the event marker in the GUI
   */
  public EventTrigger(String name) {
    super(EventScheduler.getEventLoop(), pollCondition(name));
  }

  /**
   * Create a new EventTrigger that gets polled by the given event loop instead of the
   * EventScheduler
   *
   * @param eventLoop The event loop to poll this trigger
   * @param name The name of the event. This will be the name of the event marker in the GUI
   */
  public EventTrigger(EventLoop eventLoop, String name) {
    super(eventLoop, pollCondition(name));
  }

  /**
   * Create a boolean supplier that will poll a condition.
   *
   * @param name The name of the event
   * @return A boolean supplier to poll the event's condition
   */
  private static BooleanSupplier pollCondition(String name) {
    return () -> EventConditions.isEventActive(name);
  }

  /**
   * Set the value of an event condition
   *
   * @param name The name of the condition
   * @param value The value of the condition
   */
  protected static void setCondition(String name, boolean value) {
    EventConditions.setEventActive(name, value);
  }
}
