package com.pathplanner.lib.command3;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
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
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.RobotController;

/** Runs a Commands v3 scheduler with a simulated clock that advances 20ms per loop. */
abstract class CommandsV3TestBase {
  static final double LOOP_PERIOD = 0.02;

  static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          60.0,
          6.0,
          new ModuleConfig(0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
          new Translation2d(0.3, 0.3),
          new Translation2d(0.3, -0.3),
          new Translation2d(-0.3, 0.3),
          new Translation2d(-0.3, -0.3));

  /** A mechanism that is only used for requirements */
  static final class TestMechanism implements Mechanism {
    private final String name;
    private final Scheduler scheduler;

    TestMechanism(String name, Scheduler scheduler) {
      this.name = name;
      this.scheduler = scheduler;
    }

    @Override
    public String getName() {
      return name;
    }

    @Override
    public Scheduler getRegisteredScheduler() {
      return scheduler;
    }
  }

  protected Scheduler scheduler;
  protected TestMechanism drive;
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
    drive = new TestMechanism("Drive", scheduler);
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
        ROBOT_CONFIG,
        () -> false,
        drive);
  }

  /** A straight 3 meter path along the X axis, with the given event markers */
  static PathPlannerPath straightPath(String name, EventMarker... markers) {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(0.0, 0.0, Rotation2d.ZERO), new Pose2d(3.0, 0.0, Rotation2d.ZERO)),
            List.of(),
            List.of(),
            List.of(),
            List.of(markers),
            new PathConstraints(3.0, 3.0, 6.0, 6.0),
            new IdealStartingState(0.0, Rotation2d.ZERO),
            new GoalEndState(0.0, Rotation2d.ZERO),
            false);
    path.name = name;
    return path;
  }

  /** A command that requires the given mechanism and runs until it is canceled */
  static Command runForever(Mechanism mechanism, String name) {
    return mechanism.run(coroutine -> coroutine.park()).named(name);
  }
}
