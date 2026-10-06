package com.pathplanner.lib.commands;

import static com.pathplanner.lib.TestFixtures.straightPath;
import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.TestFixtures;
import com.pathplanner.lib.auto.CommandSpec;
import com.pathplanner.lib.auto.CommandSpec.GroupType;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.PPLibTesting;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.*;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.RobotController;

/** Tests that the Commands v2 API still works on top of the framework-independent core */
class FollowPathCommandTest {
  private long timeNanos;
  private final Subsystem drive = new Subsystem() {};
  private final Subsystem intake = new Subsystem() {};

  @BeforeEach
  void setUp() {
    timeNanos = 1_000_000_000L;
    RobotController.setTimeSource(() -> timeNanos);
    PPLibTesting.resetForTesting();
  }

  private FollowPathCommand followPathCommand(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        () -> Pose2d.ZERO,
        ChassisVelocities::new,
        (_, _) -> {},
        new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
        TestFixtures.ROBOT_CONFIG,
        () -> false,
        drive);
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
            straightPath(
                "Straight", new EventMarker("Intake", 0.2, 0.6, CommandSpec.of(markerCommand))));

    assertTrue(command.getRequirements().containsAll(List.of(drive, intake)));

    command.initialize();
    assertEquals("Straight", ActivePathState.getCurrentPathName());
    assertEquals("Straight", PathPlannerAuto.currentPathName);
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
    assertEquals("", PathPlannerAuto.currentPathName);
  }

  @Test
  void markerCommandsAreBuiltWhenThePathCommandIsCreated() {
    AtomicInteger registeredRuns = new AtomicInteger();
    AtomicInteger replacementRuns = new AtomicInteger();
    NamedCommands.registerCommand(
        "Intake", Commands.runOnce(registeredRuns::incrementAndGet, intake));
    FollowPathCommand command =
        followPathCommand(
            straightPath(
                "Straight", new EventMarker("Intake", 0.5, new CommandSpec.Named("Intake"))));

    // The marker should run the command that was registered when the path command was created
    NamedCommands.registerCommand(
        "Intake", Commands.runOnce(replacementRuns::incrementAndGet, intake));
    command.initialize();
    for (int loops = 0; !command.isFinished() && loops < 1000; loops++) {
      timeNanos += 20_000_000L;
      command.execute();
    }
    command.end(false);

    assertEquals(1, registeredRuns.get());
    assertEquals(0, replacementRuns.get());
  }

  @Test
  void markerCommandsCannotRequireDrive() {
    Command driveCommand = Commands.idle(drive);
    assertThrows(
        IllegalArgumentException.class,
        () ->
            followPathCommand(
                straightPath(
                    "Straight", new EventMarker("Drive", 0.5, CommandSpec.of(driveCommand)))));
  }

  @Test
  void buildsCommandGroupsFromSpecs() throws Exception {
    assertInstanceOf(
        SequentialCommandGroup.class,
        CommandUtil.buildCommand(
            new CommandSpec.Group(GroupType.SEQUENTIAL, List.of(new CommandSpec.Wait(1.0))),
            false));
    assertInstanceOf(
        ParallelCommandGroup.class,
        CommandUtil.buildCommand(
            new CommandSpec.Group(GroupType.PARALLEL, List.of(new CommandSpec.None())), false));
    assertInstanceOf(
        ParallelRaceGroup.class,
        CommandUtil.buildCommand(
            new CommandSpec.Group(GroupType.RACE, List.of(new CommandSpec.None())), false));
    assertInstanceOf(
        ParallelDeadlineGroup.class,
        CommandUtil.buildCommand(
            new CommandSpec.Group(
                GroupType.DEADLINE, List.of(new CommandSpec.Wait(1.0), new CommandSpec.None())),
            false));
    assertInstanceOf(WaitCommand.class, CommandUtil.buildCommand(new CommandSpec.Wait(1.0), false));
    assertThrows(
        IllegalArgumentException.class,
        () -> CommandUtil.buildCommand(CommandSpec.of("not a command"), false));
  }
}
