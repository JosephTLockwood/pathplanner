package com.pathplanner.lib.command3;

import com.pathplanner.lib.TestFixtures;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTesting;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.MockOpModes;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.RobotController;

/** Runs a Commands v3 scheduler with a simulated clock that advances 20ms per loop. */
abstract class CommandsV3TestBase {
  static final double LOOP_PERIOD = 0.02;

  protected Scheduler scheduler;
  protected Mechanism drive;
  protected Pose2d robotPose;
  protected final List<ChassisVelocities> outputs = new ArrayList<>();
  protected final BiConsumer<ChassisVelocities, DriveFeedforwards> output =
      (speeds, _) -> outputs.add(speeds);
  private long timeNanos;

  @BeforeEach
  void setUpScheduler() {
    timeNanos = 1_000_000_000L;
    RobotController.setTimeSource(() -> timeNanos);
    MockOpModes.install();
    PPLibTesting.resetForTesting();

    scheduler = Scheduler.createIndependentScheduler();
    drive = new Mechanism() {};
    robotPose = Pose2d.ZERO;
    outputs.clear();
  }

  @AfterEach
  void tearDownScheduler() {
    scheduler.cancelAll();
    PPLibTesting.resetForTesting();
  }

  /** Advance time by one loop, then run the scheduler */
  protected void step() {
    timeNanos += (long) (LOOP_PERIOD * 1e9);
    scheduler.run();
  }

  /**
   * Run the scheduler until the condition is true
   *
   * @return The number of loops that were run
   */
  protected int stepUntil(BooleanSupplier condition, int maxLoops) {
    int loops = 0;
    while (!condition.getAsBoolean()) {
      if (loops >= maxLoops) {
        throw new AssertionError("Condition was not met within " + maxLoops + " loops");
      }
      step();
      loops++;
    }
    return loops;
  }

  protected void stepFor(double seconds) {
    for (int i = 0; i < Math.round(seconds / LOOP_PERIOD); i++) {
      step();
    }
  }

  protected boolean isRunning(Command command) {
    return scheduler.isScheduledOrRunning(command);
  }

  protected FollowPathCommand followPathCommand(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        () -> robotPose,
        ChassisVelocities::new,
        output,
        new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
        TestFixtures.ROBOT_CONFIG,
        () -> false,
        drive);
  }

  /** A command that requires the given mechanism and runs until it is canceled */
  static Command runForever(Mechanism mechanism, String name) {
    return mechanism.run(coroutine -> coroutine.park()).named(name);
  }
}
