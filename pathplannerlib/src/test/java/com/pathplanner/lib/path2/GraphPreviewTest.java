package com.pathplanner.lib.path2;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PPLibTesting;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Runs each saved path through {@link GraphFollower} on a robot that moves exactly as commanded,
 * which is how the app's preview simulates it, and compares the result with the app's preview.
 */
class GraphPreviewTest {
  /** The app shows preview times to two decimals, and a path ends on a 20 ms loop */
  private static final double TIME_TOLERANCE = 0.02;

  private FixtureProject project;
  private RobotConfig config;

  @BeforeEach
  void openProject() throws Exception {
    PPLibTesting.resetForTesting();
    project = FixtureProject.open2027();
    config = RobotConfig.fromGUISettings();
  }

  @AfterEach
  void closeProject() {
    project.close();
    PPLibTesting.resetForTesting();
  }

  /** The result of following a path the way the app previews it */
  private record Run(
      double seconds, Pose2d finalPose, List<String> events, List<Pose2d> eventPoses) {}

  private Run follow(PathGraph path) {
    GraphWaypoint first = path.getStartNode().waypoint();
    Pose2d[] pose = {
      new Pose2d(first.position(), first.isPose() ? first.rotation() : Rotation2d.ZERO)
    };
    ChassisVelocities[] speeds = {new ChassisVelocities()};
    List<String> events = new ArrayList<>();
    List<Pose2d> eventPoses = new ArrayList<>();

    GraphFollower follower =
        new GraphFollower(
            path,
            () -> pose[0],
            () -> speeds[0],
            (output, _) -> speeds[0] = output,
            config,
            () -> false,
            name -> {
              events.add(name);
              eventPoses.add(pose[0]);
            });

    follower.start();
    int steps = 0;
    while (true) {
      steps++;
      assertTrue(steps < 6000, "The path did not finish within 120 seconds");
      follower.follow();
      pose[0] =
          pose[0].plus(
              new Twist2d(
                      speeds[0].vx * GraphController.PERIOD_SECONDS,
                      speeds[0].vy * GraphController.PERIOD_SECONDS,
                      speeds[0].omega * GraphController.PERIOD_SECONDS)
                  .exp());
      if (follower.isFinished()) {
        break;
      }
    }
    follower.stop(false);
    return new Run(steps * GraphController.PERIOD_SECONDS, pose[0], events, eventPoses);
  }

  private static void assertEndsAt(PathGraph.Node end, Pose2d pose, Rotation2d heading) {
    assertTrue(
        pose.getTranslation().getDistance(end.waypoint().position()) < end.endToleranceMeters(),
        "Ended at " + pose + ", not within tolerance of " + end.waypoint().position());
    if (heading != null) {
      assertEquals(
          0.0,
          pose.getRotation().minus(heading).getRadians(),
          end.endToleranceRadians(),
          "Ended facing " + pose.getRotation() + " instead of " + heading);
    }
  }

  private static PathGraph.Node lastNode(PathGraph path) {
    PathGraph.Node node = path.getStartNode();
    while (!path.getOutgoingBranches(node.id()).isEmpty()) {
      node = path.getNode(path.getOutgoingBranches(node.id()).get(0).targetId());
    }
    return node;
  }

  @Test
  void straightMatchesPreview() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Straight");
    Run run = follow(path);
    assertEquals(FixtureProject.STRAIGHT_PREVIEW_SECONDS, run.seconds(), TIME_TOLERANCE);
    PathGraph.Node end = lastNode(path);
    assertEndsAt(end, run.finalPose(), end.waypoint().rotation());
  }

  @Test
  void leaveStartMatchesPreview() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Leave Start");
    Run run = follow(path);
    assertEquals(FixtureProject.LEAVE_START_PREVIEW_SECONDS, run.seconds(), TIME_TOLERANCE);
    PathGraph.Node end = lastNode(path);
    assertEndsAt(end, run.finalPose(), Rotation2d.fromDegrees(-135.0));
  }

  @Test
  void rotateEventsMatchesPreview() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Rotate Events");
    Run run = follow(path);
    assertEquals(FixtureProject.ROTATE_EVENTS_PREVIEW_SECONDS, run.seconds(), TIME_TOLERANCE);

    // The end waypoint is a translation waypoint, so the robot keeps the heading it had when it
    // moved on from the 90 degree pose waypoint
    PathGraph.Node end = lastNode(path);
    assertEndsAt(end, run.finalPose(), null);
    assertEquals(90.0, run.finalPose().getRotation().getDegrees(), 5.0);
  }

  @Test
  void rotateEventsSignalsEventsWhereTheAppPlacesThem() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Rotate Events");
    Run run = follow(path);
    assertEquals(List.of("Intake", "Score"), run.events());

    // "Intake" sits at 50% of the first branch: the app places it where the robot's distance to the
    // branch's target is half the branch length
    Translation2d source = new Translation2d(-6.0, 3.0);
    Translation2d target = new Translation2d(-3.0, 1.5);
    double branchLength = source.getDistance(target);
    double ratio = run.eventPoses().get(0).getTranslation().getDistance(target) / branchLength;
    assertTrue(ratio <= 0.5, "Intake fired early, at ratio " + ratio);
    assertTrue(ratio > 0.45, "Intake fired more than a loop late, at ratio " + ratio);

    // "Score" is on the 90 degree waypoint, which the robot moves on from 0.5 m before it
    double distanceToScore = run.eventPoses().get(1).getTranslation().getDistance(target);
    assertTrue(distanceToScore <= 0.5, "Score fired " + distanceToScore + " m from its waypoint");
  }

  @Test
  void flippedPathEndsOnTheOtherSide() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Leave Start");
    Run blue = follow(path);
    Run red = follow(path.flip());
    assertEquals(blue.seconds(), red.seconds(), TIME_TOLERANCE);
    assertEquals(-blue.finalPose().getX(), red.finalPose().getX(), 0.02);
    assertEquals(-blue.finalPose().getY(), red.finalPose().getY(), 0.02);
  }
}
