package com.pathplanner.lib.command3;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.events.EventConditions;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;

class PathPlannerAutoTest extends CommandsV3TestBase {
  private static Command counter(AtomicInteger count) {
    return Command.noRequirements(_ -> count.incrementAndGet()).named("Count");
  }

  private PathPlannerAuto autoThatWaits(double seconds) {
    return new PathPlannerAuto(
        scheduler, Command.waitFor(Seconds.of(seconds)).named("Auto Body"), Pose2d.ZERO);
  }

  @Test
  void runsAutoCommandAsChild() {
    var intake = new TestMechanism("Intake", scheduler);
    Command body = runForever(intake, "Body");
    var auto = new PathPlannerAuto(scheduler, body, Pose2d.ZERO);

    assertEquals("Body", auto.name());
    assertEquals(body.requirements(), auto.requirements());

    scheduler.schedule(auto);
    step();
    assertTrue(isRunning(body));
    assertSame(auto, scheduler.getParentOf(body));

    scheduler.cancel(auto);
    assertFalse(isRunning(body));
  }

  @Test
  void isRunningTriggerFollowsAuto() {
    var auto = autoThatWaits(0.5);
    AtomicInteger started = new AtomicInteger();
    AtomicInteger stopped = new AtomicInteger();
    auto.isRunning().onTrue(counter(started)).onFalse(counter(stopped));

    // Commands v3 triggers treat their first poll as an edge, so onFalse runs once here
    stepFor(0.1);
    assertEquals(0, started.get());
    int stoppedBeforeAuto = stopped.get();

    scheduler.schedule(auto);
    stepFor(0.1);
    assertEquals(1, started.get());
    assertEquals(stoppedBeforeAuto, stopped.get());

    stepUntil(() -> !isRunning(auto), 100);
    step();
    assertEquals(stoppedBeforeAuto + 1, stopped.get());
  }

  @Test
  void conditionTriggersAreOnlyActiveWhileRunning() {
    var auto = autoThatWaits(0.5);
    AtomicBoolean condition = new AtomicBoolean(true);
    AtomicInteger fired = new AtomicInteger();
    auto.condition(condition::get).onTrue(counter(fired));

    stepFor(0.2);
    assertEquals(0, fired.get(), "Auto triggers should not fire before the auto runs");

    scheduler.schedule(auto);
    stepFor(0.1);
    assertEquals(1, fired.get());

    stepUntil(() -> !isRunning(auto), 100);
    stepFor(0.2);
    assertEquals(1, fired.get());
  }

  @Test
  void timeElapsedTrigger() {
    var auto = autoThatWaits(1.0);
    AtomicInteger fired = new AtomicInteger();
    auto.timeElapsed(0.5).onTrue(counter(fired));

    scheduler.schedule(auto);
    stepFor(0.4);
    assertEquals(0, fired.get());

    stepFor(0.2);
    assertEquals(1, fired.get());
  }

  @Test
  void eventTriggerIgnoresEventsWhileNotRunning() {
    var auto = autoThatWaits(0.5);
    AtomicInteger fired = new AtomicInteger();
    auto.event("Shoot").onTrue(counter(fired));

    EventConditions.pulseEvent("Shoot");
    stepFor(0.1);

    // Starting the auto should not fire the trigger for the event that happened before it started
    scheduler.schedule(auto);
    stepFor(0.1);
    assertEquals(0, fired.get());

    EventConditions.pulseEvent("Shoot");
    stepFor(0.1);
    assertEquals(1, fired.get());
  }

  @Test
  void eventTriggerSeesEventsInFinalLoopOfAuto() {
    // The auto pulses the event in the same loop that it finishes
    Command body =
        Command.noRequirements(
                coroutine -> {
                  coroutine.wait(Seconds.of(0.2));
                  EventConditions.pulseEvent("Score");
                })
            .named("Auto Body");
    var auto = new PathPlannerAuto(scheduler, body, Pose2d.ZERO);
    AtomicInteger fired = new AtomicInteger();
    auto.event("Score").onTrue(counter(fired));

    scheduler.schedule(auto);
    stepUntil(() -> !isRunning(auto), 100);
    stepFor(0.1);
    assertEquals(1, fired.get());
  }

  @Test
  void pointTowardsZoneTrigger() {
    var auto = autoThatWaits(1.0);
    AtomicBoolean inZone = new AtomicBoolean();
    auto.pointTowardsZone("Speaker")
        .whileTrue(
            Command.noRequirements(
                    coroutine -> {
                      inZone.set(true);
                      coroutine.park();
                    })
                .whenCanceled(() -> inZone.set(false))
                .named("In Zone"));

    scheduler.schedule(auto);
    stepFor(0.1);
    assertFalse(inZone.get());

    EventConditions.setWithinZone("Speaker", true);
    stepFor(0.1);
    assertTrue(inZone.get());

    EventConditions.setWithinZone("Speaker", false);
    stepFor(0.1);
    assertFalse(inZone.get());
  }
}
