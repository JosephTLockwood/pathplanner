package com.pathplanner.lib.command3;

import static com.pathplanner.lib.TestFixtures.straightPath;
import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.TestFixtures;
import com.pathplanner.lib.auto.CommandSpec;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.kinematics.ChassisVelocities;

class FollowPathCommandTest extends CommandsV3TestBase {
  @Test
  void followsPathUntilTrajectoryEnds() {
    PathPlannerPath path = straightPath("Straight");
    double totalTime =
        path.getIdealTrajectory(TestFixtures.ROBOT_CONFIG).orElseThrow().getTotalTimeSeconds();
    FollowPathCommand command = followPathCommand(path);

    scheduler.schedule(command);
    step();
    assertTrue(isRunning(command));
    assertEquals("Straight", ActivePathState.getCurrentPathName());
    assertNotNull(ActivePathState.getCurrentTrajectory());

    int loops = stepUntil(() -> !isRunning(command), 1000);
    assertEquals(totalTime, (loops + 1) * LOOP_PERIOD, 2 * LOOP_PERIOD);

    // The path ends stopped, so zero speeds should be output when it finishes
    ChassisVelocities lastOutput = outputs.get(outputs.size() - 1);
    assertEquals(0.0, Math.hypot(lastOutput.vx, lastOutput.vy), 1e-9);
    assertEquals("", ActivePathState.getCurrentPathName());
    assertNull(ActivePathState.getCurrentTrajectory());
  }

  @Test
  void canceledPathDoesNotOutputZeroSpeeds() {
    FollowPathCommand command = followPathCommand(straightPath("Straight"));

    scheduler.schedule(command);
    stepFor(0.5);
    int outputCount = outputs.size();

    scheduler.cancel(command);
    assertFalse(isRunning(command));
    assertEquals(outputCount, outputs.size());
    assertEquals("", ActivePathState.getCurrentPathName());
  }

  @Test
  void markerCommandsAreForkedAndCanceledWhenPathEnds() {
    var intake = new Mechanism() {};
    Command intakeCommand = runForever(intake, "Run Intake");
    PathPlannerPath path =
        straightPath("Markers", new EventMarker("Intake", 0.5, CommandSpec.of(intakeCommand)));
    FollowPathCommand command = followPathCommand(path);

    scheduler.schedule(command);
    step();
    assertFalse(isRunning(intakeCommand));

    stepUntil(() -> isRunning(intakeCommand), 200);
    assertTrue(isRunning(command));
    // The marker command doesn't require the drive, so it is not a requirement of the path command
    assertFalse(command.requirements().contains(intake));

    stepUntil(() -> !isRunning(command), 1000);
    assertFalse(isRunning(intakeCommand), "Marker commands should be canceled when the path ends");
  }

  @Test
  void zonedMarkerCommandIsCanceledAtEndOfZone() {
    var intake = new Mechanism() {};
    Command intakeCommand = runForever(intake, "Run Intake");
    PathPlannerPath path =
        straightPath("Zone", new EventMarker("Intake", 0.2, 0.6, CommandSpec.of(intakeCommand)));
    FollowPathCommand command = followPathCommand(path);

    scheduler.schedule(command);
    stepUntil(() -> isRunning(intakeCommand), 200);
    stepUntil(() -> !isRunning(intakeCommand), 200);
    assertTrue(isRunning(command), "The path should still be running after the zone ends");
  }

  @Test
  void markerCommandsCannotRequireTheDrive() {
    Command driveCommand = runForever(drive, "Drive Command");
    PathPlannerPath path =
        straightPath("Bad", new EventMarker("Drive", 0.5, CommandSpec.of(driveCommand)));

    assertThrows(IllegalArgumentException.class, () -> followPathCommand(path));
  }

  @Test
  void oneShotEventTriggerFiresOncePerMarker() {
    AtomicInteger fired = new AtomicInteger();
    new EventTrigger(scheduler, "Shoot")
        .onTrue(Command.noRequirements(_ -> fired.incrementAndGet()).named("Count"));

    FollowPathCommand command =
        followPathCommand(straightPath("Shoot", new EventMarker("Shoot", 0.5)));

    scheduler.schedule(command);
    stepUntil(() -> !isRunning(command), 1000);
    stepFor(0.1);
    assertEquals(1, fired.get());

    // Running the path again should fire the event again
    scheduler.schedule(command);
    stepUntil(() -> !isRunning(command), 1000);
    stepFor(0.1);
    assertEquals(2, fired.get());
  }

  @Test
  void eventTriggerCommandsAreNotCanceledWhenPathEnds() {
    var shooter = new Mechanism() {};
    Command shoot = runForever(shooter, "Shoot");
    new EventTrigger(scheduler, "Shoot").onTrue(shoot);

    FollowPathCommand command =
        followPathCommand(straightPath("Shoot", new EventMarker("Shoot", 0.9)));

    scheduler.schedule(command);
    stepUntil(() -> !isRunning(command), 1000);
    stepFor(0.1);
    assertTrue(isRunning(shoot), "Commands bound to event triggers run independently of the path");
  }

  @Test
  void zonedEventTriggerIsActiveDuringZone() {
    var intake = new Mechanism() {};
    Command intakeCommand = runForever(intake, "Run Intake");
    new EventTrigger(scheduler, "Intake").whileTrue(intakeCommand);

    FollowPathCommand command =
        followPathCommand(straightPath("Zone", new EventMarker("Intake", 0.2, 0.6)));

    scheduler.schedule(command);
    stepUntil(() -> isRunning(intakeCommand), 200);
    stepUntil(() -> !isRunning(intakeCommand), 200);
    assertTrue(isRunning(command));
  }

  @Test
  void zonedEventIsDeactivatedWhenPathIsCanceled() {
    var intake = new Mechanism() {};
    Command intakeCommand = runForever(intake, "Run Intake");
    new EventTrigger(scheduler, "Intake").whileTrue(intakeCommand);

    FollowPathCommand command =
        followPathCommand(straightPath("Zone", new EventMarker("Intake", 0.2, 0.9)));

    scheduler.schedule(command);
    stepUntil(() -> isRunning(intakeCommand), 200);

    scheduler.cancel(command);
    stepFor(0.1);
    assertFalse(isRunning(intakeCommand));
  }
}
