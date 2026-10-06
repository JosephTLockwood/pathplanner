package com.pathplanner.lib.events;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Time;

/** Event that will activate a trigger, then deactivate it the next loop */
public class OneShotTriggerEvent extends Event {

  private final String name;

  /**
   * Create an event for activating a trigger, then deactivating it the next loop
   *
   * @param timestamp The trajectory timestamp of this event
   * @param name The name of the trigger to control
   */
  public OneShotTriggerEvent(double timestamp, String name) {
    super(timestamp);
    this.name = name;
  }

  /**
   * Create an event for activating a trigger, then deactivating it the next loop
   *
   * @param timestamp The trajectory timestamp of this event
   * @param name The name of the trigger to control
   */
  public OneShotTriggerEvent(Time timestamp, String name) {
    this(timestamp.in(Seconds), name);
  }

  /**
   * Get the event name for this event
   *
   * @return The event name
   */
  public String getEventName() {
    return name;
  }

  @Override
  public void handleEvent(EventSchedulerBase eventScheduler) {
    eventScheduler.handleOneShotTrigger(name);
  }

  @Override
  public void cancelEvent(EventSchedulerBase eventScheduler) {
    // Do nothing
  }

  @Override
  public Event copyWithTimestamp(double timestampSeconds) {
    return new OneShotTriggerEvent(timestampSeconds, name);
  }
}
