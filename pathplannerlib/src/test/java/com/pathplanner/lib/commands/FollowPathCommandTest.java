package com.pathplanner.lib.commands;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.CommandSpec;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.PPLibTesting;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.*;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.RobotController;

/** Tests that the Commands v2 API still works on top of the framework-independent core */
class FollowPathCommandTest {
  private static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          60.0,
          6.0,
          new ModuleConfig(0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
          new Translation2d(0.3, 0.3),
          new Translation2d(0.3, -0.3),
          new Translation2d(-0.3, 0.3),
          new Translation2d(-0.3, -0.3));

  private long timeNanos;
  private final Subsystem drive = new Subsystem() {};
  private final Subsystem intake = new Subsystem() {};

  @BeforeEach
  void setUp() {
    timeNanos = 1_000_000_000L;
    RobotController.setTimeSource(() -> timeNanos);
    PPLibTesting.resetForTesting();
  }

  @AfterEach
  void tearDown() {
    PPLibTesting.resetForTesting();
  }

  private FollowPathCommand followPathCommand(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        () -> Pose2d.ZERO,
        ChassisVelocities::new,
        (_, _) -> {},
        new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
        ROBOT_CONFIG,
        () -> false,
        drive);
  }

  private static PathPlannerPath straightPath(EventMarker... markers) {
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
    path.name = "Straight";
    return path;
  }

  @Test
  void runsZonedMarkerCommandsWithinPath() {
    AtomicInteger initialized = new AtomicInteger();
    AtomicInteger executed = new AtomicInteger();
    AtomicInteger interrupted = new AtomicInteger();
    Command markerCommand =
        new FunctionalCommand(
            initialized::incrementAndGet,
            executed::incrementAndGet,
            wasInterrupted -> {
              if (wasInterrupted) {
                interrupted.incrementAndGet();
              }
            },
            () -> false,
            intake);
    FollowPathCommand command =
        followPathCommand(
            straightPath(new EventMarker("Intake", 0.2, 0.6, CommandSpec.of(markerCommand))));

    assertTrue(command.getRequirements().containsAll(List.of(drive, intake)));

    command.initialize();
    assertEquals("Straight", ActivePathState.getCurrentPathName());
    int loops = 0;
    do {
      timeNanos += 20_000_000L;
      command.execute();
      loops++;
    } while (!command.isFinished() && loops < 1000);
    command.end(false);

    assertTrue(command.isFinished());
    assertEquals(1, initialized.get());
    assertTrue(executed.get() > 0);
    assertEquals(1, interrupted.get(), "The marker command should end at the end of its zone");
    assertEquals("", ActivePathState.getCurrentPathName());
  }

  @Test
  void markerCommandsCannotRequireDrive() {
    Command driveCommand = Commands.idle(drive);
    assertThrows(
        IllegalArgumentException.class,
        () ->
            followPathCommand(
                straightPath(new EventMarker("Drive", 0.5, CommandSpec.of(driveCommand)))));
  }

  @Test
  void buildsCommandGroupsFromSpecs() throws Exception {
    assertInstanceOf(
        SequentialCommandGroup.class,
        CommandUtil.buildCommand(
            new CommandSpec.Sequential(List.of(new CommandSpec.Wait(1.0))), false));
    assertInstanceOf(
        ParallelCommandGroup.class,
        CommandUtil.buildCommand(new CommandSpec.Parallel(List.of(new CommandSpec.None())), false));
    assertInstanceOf(
        ParallelRaceGroup.class,
        CommandUtil.buildCommand(new CommandSpec.Race(List.of(new CommandSpec.None())), false));
    assertInstanceOf(
        ParallelDeadlineGroup.class,
        CommandUtil.buildCommand(
            new CommandSpec.Deadline(List.of(new CommandSpec.Wait(1.0), new CommandSpec.None())),
            false));
    assertInstanceOf(WaitCommand.class, CommandUtil.buildCommand(new CommandSpec.Wait(1.0), false));
    assertThrows(
        IllegalArgumentException.class,
        () -> CommandUtil.buildCommand(CommandSpec.of("not a command"), false));
  }
}
