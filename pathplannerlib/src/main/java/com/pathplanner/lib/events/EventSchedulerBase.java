package com.pathplanner.lib.events;

import com.pathplanner.lib.auto.CommandSpec;

/**
 * Base class for the schedulers that handle {@link Event events} while following a trajectory.
 *
 * <p>This class does not depend on a command framework. Each supported command framework provides
 * its own implementation that decides how commands are run:
 *
 * <ul>
 *   <li>Commands v2: {@link EventScheduler}
 *   <li>Commands v3: {@link com.pathplanner.lib.command3.EventScheduler}
 * </ul>
 */
public abstract class EventSchedulerBase {
  /** Create a new event scheduler */
  protected EventSchedulerBase() {}

  /**
   * Schedule a command on this scheduler. This will cancel other commands that share requirements
   * with the given command.
   *
   * @param command The command to schedule
   */
  protected abstract void scheduleCommand(CommandSpec command);

  /**
   * Cancel a command on this scheduler.
   *
   * @param command The command to cancel
   */
  protected abstract void cancelCommand(CommandSpec command);

  /**
   * Activate an event trigger for a single loop, then deactivate it
   *
   * @param eventName The name of the event
   */
  protected abstract void handleOneShotTrigger(String eventName);
}
