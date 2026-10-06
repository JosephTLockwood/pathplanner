package com.pathplanner.lib.command3;

import com.pathplanner.lib.events.EventConditions;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;

/**
 * A Commands v3 trigger that will be controlled by the robot entering/leaving a point towards zone
 */
public class PointTowardsZoneTrigger extends Trigger {
  /**
   * Create a new PointTowardsZoneTrigger that is polled by the default scheduler
   *
   * @param name The name of the point towards zone
   */
  public PointTowardsZoneTrigger(String name) {
    this(Scheduler.getDefault(), name);
  }

  /**
   * Create a new PointTowardsZoneTrigger that is polled by the given scheduler
   *
   * @param scheduler The scheduler that polls this trigger and runs its bound commands
   * @param name The name of the point towards zone
   */
  public PointTowardsZoneTrigger(Scheduler scheduler, String name) {
    super(scheduler, () -> EventConditions.isWithinZone(name));
  }
}
