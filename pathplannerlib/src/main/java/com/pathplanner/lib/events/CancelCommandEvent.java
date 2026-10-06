package com.pathplanner.lib.events;

import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.CommandSpec;
import org.wpilib.units.measure.Time;

/** Event that will cancel a command within the EventScheduler */
public class CancelCommandEvent extends Event {

  private final CommandSpec command;

  /**
   * Create an event to cancel a command
   *
   * @param timestamp The trajectory timestamp for this event
   * @param command The command to cancel
   */
  public CancelCommandEvent(double timestamp, CommandSpec command) {
    super(timestamp);
    this.command = command;
  }

  /**
   * Create an event to cancel a command
   *
   * @param timestamp The trajectory timestamp for this event
   * @param command The command to cancel
   */
  public CancelCommandEvent(Time timestamp, CommandSpec command) {
    this(timestamp.in(Seconds), command);
  }

  @Override
  public void handleEvent(EventSchedulerBase eventScheduler) {
    eventScheduler.cancelCommand(command);
  }

  @Override
  public void cancelEvent(EventSchedulerBase eventScheduler) {
    // Do nothing, the event scheduler will already cancel all commands
  }

  @Override
  public Event copyWithTimestamp(double timestampSeconds) {
    return new CancelCommandEvent(timestampSeconds, command);
  }
}
