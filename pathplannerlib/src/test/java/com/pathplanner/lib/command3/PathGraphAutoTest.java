package com.pathplanner.lib.command3;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.NamedConditions;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path2.FixtureProject;
import com.pathplanner.lib.path2.PathGraph;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

class PathGraphAutoTest extends CommandsV3TestBase {
  private FixtureProject project;
  private ChassisVelocities speeds;
  private final List<Pose2d> resets = new ArrayList<>();

  @BeforeEach
  void configureAutoBuilder() throws Exception {
    project = FixtureProject.open2027();
    speeds = new ChassisVelocities();
    resets.clear();
    AutoBuilder.configure(
        () -> robotPose,
        pose -> {
          resets.add(pose);
          robotPose = pose;
        },
        () -> speeds,
        (newSpeeds, _) -> {
          // The robot moves exactly as commanded, as in the app's preview
          speeds = newSpeeds;
          outputs.add(newSpeeds);
          robotPose =
              robotPose.plus(
                  new Twist2d(
                          newSpeeds.vx * LOOP_PERIOD,
                          newSpeeds.vy * LOOP_PERIOD,
                          newSpeeds.omega * LOOP_PERIOD)
                      .exp());
        },
        new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
        RobotConfig.fromGUISettings(),
        () -> false,
        drive);
  }

  @AfterEach
  void closeProject() {
    project.close();
  }

  private static Command counter(String name, AtomicInteger count) {
    return Command.noRequirements(_ -> count.incrementAndGet()).named(name);
  }

  private static boolean isZero(ChassisVelocities speeds) {
    return speeds.vx == 0.0 && speeds.vy == 0.0 && speeds.omega == 0.0;
  }

  @Test
  void autoResetsOdometryFollowsPathThenRunsCommand() {
    AtomicInteger hello = new AtomicInteger();
    AtomicInteger spinup = new AtomicInteger();
    NamedCommands.registerCommand("PrintHello", counter("PrintHello", hello));
    new EventTrigger(scheduler, "Spinup").onTrue(counter("Spinup", spinup));

    var auto = new PathPlannerAuto(scheduler, "Fixture Auto", false);
    assertEquals(new Pose2d(14.27, 1.035, Rotation2d.PI), auto.getStartingPose());
    assertTrue(auto.requirements().contains(drive));

    robotPose = new Pose2d(1.0, 1.0, Rotation2d.CCW_90DEG);
    scheduler.schedule(auto);
    int loops = stepUntil(() -> !isRunning(auto), 500);

    assertEquals(List.of(new Pose2d(14.27, 1.035, Rotation2d.PI)), resets);
    assertEquals(1, hello.get());
    assertEquals(1, spinup.get());
    assertEquals(0.0, robotPose.getTranslation().getDistance(new Translation2d(12.27, 2.035)), 0.1);
    assertTrue(isZero(outputs.getLast()), "A finished path should send zero speeds");
    // The app previews this path at 1.36 s. The auto adds the reset and command loops.
    assertEquals(FixtureProject.STRAIGHT_PREVIEW_SECONDS, loops * LOOP_PERIOD, 0.1);
  }

  @Test
  void leaveStartDrivesToItsEndPose() {
    var auto = new PathPlannerAuto(scheduler, "Leave Start", false);
    scheduler.schedule(auto);
    stepUntil(() -> !isRunning(auto), 500);

    assertEquals(1, resets.size());
    assertEquals(new Pose2d(1.02, 2.035, Rotation2d.ZERO), resets.get(0));
    assertEquals(0.0, robotPose.getTranslation().getDistance(new Translation2d(3.02, 2.035)), 0.1);
    assertEquals(
        0.0, robotPose.getRotation().minus(Rotation2d.fromDegrees(45.0)).getDegrees(), 2.0);
  }

  @Test
  void canceledPathSendsNoZero() throws Exception {
    Command follow = AutoBuilder.followPath(PathGraph.fromPathFile("Leave Start"));
    robotPose = new Pose2d(1.02, 2.035, Rotation2d.ZERO);
    scheduler.schedule(follow);
    stepFor(0.4);
    int sent = outputs.size();
    assertFalse(isZero(outputs.getLast()));

    scheduler.cancel(follow);
    step();
    assertEquals(sent, outputs.size(), "A canceled path should not send anything");
  }

  @Test
  void flippedAutoResetsToTheOtherSide() {
    PPLibTestingAccess.reconfigureWithFlip(this, () -> true);
    var auto = new PathPlannerAuto(scheduler, "Leave Start", false);
    scheduler.schedule(auto);
    stepUntil(() -> !isRunning(auto), 500);

    assertEquals(
        FlippingUtil.flipFieldPose(new Pose2d(1.02, 2.035, Rotation2d.ZERO)), resets.get(0));
    assertEquals(new Pose2d(15.52, 6.035, Rotation2d.PI), resets.get(0));
    assertEquals(
        0.0,
        robotPose
            .getTranslation()
            .getDistance(FlippingUtil.flipFieldPosition(new Translation2d(3.02, 2.035))),
        0.1);
  }

  @Test
  void conditionBranchCancelsTheRunningStep() throws Exception {
    AtomicBoolean ready = new AtomicBoolean(false);
    AtomicInteger done = new AtomicInteger();
    AtomicInteger branchEvent = new AtomicInteger();
    NamedConditions.registerCondition("Ready", ready::get);
    Command waitForever = Command.noRequirements(coroutine -> coroutine.park()).named("Wait");
    NamedCommands.registerCommand("Wait", waitForever);
    NamedCommands.registerCommand("Done", counter("Done", done));
    new EventTrigger(scheduler, "Took Ready").onTrue(counter("Took Ready", branchEvent));

    var auto = new PathPlannerAuto(scheduler, "Fixture Auto", false);
    auto.hotReload(
        json(
            """
            {"version": "2027.1",
             "nodes": [
               {"id": "a", "type": "external", "commandName": "Wait", "events": [],
                "editorPosition": {"x": 0, "y": 0}},
               {"id": "b", "type": "external", "commandName": "Done", "events": [],
                "editorPosition": {"x": 0, "y": 100}}],
             "branches": [
               {"id": "ab", "sourceId": "a", "targetId": "b",
                "transition": {"type": "condition", "conditionName": "Ready",
                               "previewDistanceMeters": 0.25},
                "events": ["Took Ready"]}],
             "startingPose": {"position": {"x": 0, "y": 0}, "rotation": 0},
             "startingPoseInitialized": false,
             "folder": null}
            """));

    scheduler.schedule(auto);
    stepFor(0.2);
    assertTrue(isRunning(waitForever));
    assertEquals(0, done.get());
    assertTrue(resets.isEmpty(), "An auto without a starting pose should not reset odometry");

    ready.set(true);
    stepUntil(() -> !isRunning(auto), 10);
    assertFalse(isRunning(waitForever));
    assertEquals(1, done.get());
    step();
    assertEquals(1, branchEvent.get());
  }

  @Test
  void finishedStepWaitsForAConditionWhenThereIsNoFinishedBranch() throws Exception {
    AtomicBoolean ready = new AtomicBoolean(false);
    AtomicInteger first = new AtomicInteger();
    AtomicInteger second = new AtomicInteger();
    NamedConditions.registerCondition("Ready", ready::get);
    NamedCommands.registerCommand("First", counter("First", first));
    NamedCommands.registerCommand("Second", counter("Second", second));

    var auto = new PathPlannerAuto(scheduler, "Fixture Auto", false);
    auto.hotReload(
        json(
            """
            {"version": "2027.1",
             "nodes": [
               {"id": "a", "type": "external", "commandName": "First", "events": [],
                "editorPosition": {"x": 0, "y": 0}},
               {"id": "b", "type": "external", "commandName": "Second", "events": [],
                "editorPosition": {"x": 0, "y": 100}}],
             "branches": [
               {"id": "ab", "sourceId": "a", "targetId": "b",
                "transition": {"type": "condition", "conditionName": "Ready"},
                "events": []}],
             "startingPose": {"position": {"x": 0, "y": 0}, "rotation": 0},
             "startingPoseInitialized": false,
             "folder": null}
            """));

    scheduler.schedule(auto);
    stepFor(0.2);
    assertEquals(1, first.get());
    assertEquals(0, second.get());
    assertTrue(isRunning(auto));

    ready.set(true);
    stepUntil(() -> !isRunning(auto), 10);
    assertEquals(1, second.get());
  }

  @Test
  void legacyAutoStillRunsInTheSameProject() {
    AtomicInteger shots = new AtomicInteger();
    NamedCommands.registerCommand("Shoot", counter("Shoot", shots));
    var auto = new PathPlannerAuto(scheduler, "Shoot and Leave", false);
    assertNotNull(auto.getStartingPose());

    scheduler.schedule(auto);
    stepUntil(() -> !isRunning(auto), 1500);
    assertEquals(1, shots.get());
    assertEquals(1, resets.size());
  }

  @Test
  void autoNamesIncludeBothFormats() {
    List<String> names = AutoBuilder.getAllAutoNames();
    assertTrue(names.contains("Fixture Auto"));
    assertTrue(names.contains("Leave Start"));
    assertTrue(names.contains("Shoot and Leave"));
  }

  private static JSONObject json(String text) throws Exception {
    return (JSONObject) new JSONParser().parse(text);
  }

  /** Reconfigures AutoBuilder with a different flip supplier for one test */
  private static final class PPLibTestingAccess {
    static void reconfigureWithFlip(
        PathGraphAutoTest test, java.util.function.BooleanSupplier flip) {
      AutoBuilder.resetForTesting();
      try {
        AutoBuilder.configure(
            () -> test.robotPose,
            pose -> {
              test.resets.add(pose);
              test.robotPose = pose;
            },
            () -> test.speeds,
            (newSpeeds, _) -> {
              test.speeds = newSpeeds;
              test.outputs.add(newSpeeds);
              test.robotPose =
                  test.robotPose.plus(
                      new Twist2d(
                              newSpeeds.vx * LOOP_PERIOD,
                              newSpeeds.vy * LOOP_PERIOD,
                              newSpeeds.omega * LOOP_PERIOD)
                          .exp());
            },
            new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
            RobotConfig.fromGUISettings(),
            flip,
            test.drive);
      } catch (Exception e) {
        throw new IllegalStateException(e);
      }
    }
  }
}
