package com.pathplanner.lib.events;

import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.CommandSpec;
import org.wpilib.units.measure.Time;

/** Event that will schedule a command within the EventScheduler */
public class ScheduleCommandEvent extends Event {

  private final CommandSpec command;

  /**
   * Create an event to schedule a command
   *
   * @param timestamp The trajectory timestamp for this event
   * @param command The command to schedule
   */
  public ScheduleCommandEvent(double timestamp, CommandSpec command) {
    super(timestamp);
    this.command = command;
  }

  /**
   * Create an event to schedule a command
   *
   * @param timestamp The trajectory timestamp for this event
   * @param command The command to schedule
   */
  public ScheduleCommandEvent(Time timestamp, CommandSpec command) {
    this(timestamp.in(Seconds), command);
  }

  /**
   * Get the command that this event will schedule
   *
   * @return The command
   */
  public CommandSpec getCommand() {
    return command;
  }

  @Override
  public void handleEvent(EventSchedulerBase eventScheduler) {
    eventScheduler.scheduleCommand(command);
  }

  @Override
  public void cancelEvent(EventSchedulerBase eventScheduler) {
    // Do nothing
  }

  @Override
  public Event copyWithTimestamp(double timestampSeconds) {
    return new ScheduleCommandEvent(timestampSeconds, command);
  }
}
