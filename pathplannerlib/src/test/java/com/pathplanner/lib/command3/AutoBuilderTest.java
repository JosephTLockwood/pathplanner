package com.pathplanner.lib.command3;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.Pair;

class AutoBuilderTest extends CommandsV3TestBase {
  private static final PathConstraints CONSTRAINTS = new PathConstraints(3.0, 3.0, 6.0, 6.0);

  /** Pathfinder that drives in a straight line from the start to the goal */
  private static class StraightLinePathfinder implements Pathfinder {
    Translation2d start;
    Translation2d goal;
    boolean newPath = false;

    @Override
    public boolean isNewPathAvailable() {
      return newPath;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
      newPath = false;
      Rotation2d heading = goal.minus(start).getAngle().orElse(Rotation2d.ZERO);
      return new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(new Pose2d(start, heading), new Pose2d(goal, heading)),
          constraints,
          null,
          goalEndState);
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
      start = startPosition;
      newPath = goal != null;
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
      goal = goalPosition;
      newPath = start != null;
    }

    @Override
    public void setDynamicObstacles(
        List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {}
  }

  private final StraightLinePathfinder pathfinder = new StraightLinePathfinder();
  private final AtomicBoolean flip = new AtomicBoolean(false);
  private final AtomicReference<Pose2d> resetPose = new AtomicReference<>();
  private final List<String> activePaths = new ArrayList<>();

  @BeforeEach
  void configure() {
    Pathfinding.setPathfinder(pathfinder);
    AutoBuilder.configure(
        () -> robotPose,
        resetPose::set,
        ChassisVelocities::new,
        this::output,
        new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
        ROBOT_CONFIG,
        flip::get,
        drive);
  }

  @AfterEach
  void resetPathfinder() {
    Pathfinding.setPathfinder(null);
  }

  @Override
  protected void step() {
    super.step();
    String name = ActivePathState.getCurrentPathName();
    if (activePaths.isEmpty() || !activePaths.get(activePaths.size() - 1).equals(name)) {
      activePaths.add(name);
    }
  }

  @Test
  void followPathRequiresDrive() {
    Command command = AutoBuilder.followPath(straightPath("Path"));
    assertInstanceOf(FollowPathCommand.class, command);
    assertEquals(java.util.Set.of(drive), command.requirements());
    assertEquals("Follow Path: Path", command.name());
  }

  @Test
  void resetOdomFlipsPose() {
    Pose2d bluePose = new Pose2d(1.0, 2.0, Rotation2d.CCW_90DEG);

    Command reset = AutoBuilder.resetOdom(bluePose);
    assertTrue(reset.requirements().isEmpty());
    scheduler.schedule(reset);
    step();
    assertEquals(bluePose, resetPose.get());

    flip.set(true);
    scheduler.schedule(AutoBuilder.resetOdom(bluePose));
    step();
    assertEquals(FlippingUtil.flipFieldPose(bluePose), resetPose.get());
  }

  @Test
  void pathfindToPoseFlippedChoosesSideWhenRun() {
    Pose2d bluePose = new Pose2d(3.0, 2.0, Rotation2d.ZERO);
    Command command = AutoBuilder.pathfindToPoseFlipped(bluePose, CONSTRAINTS);
    assertEquals(java.util.Set.of(drive), command.requirements());

    // The alliance is checked when the command runs, not when it is created
    flip.set(true);
    scheduler.schedule(command);
    step();
    assertEquals(FlippingUtil.flipFieldPose(bluePose).getTranslation(), pathfinder.goal);
  }

  @Test
  void pathfindThenFollowPathRunsEachStage() {
    PathPlannerPath goalPath =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(4.0, 2.0, Rotation2d.ZERO), new Pose2d(6.0, 2.0, Rotation2d.ZERO)),
            CONSTRAINTS,
            new IdealStartingState(0.0, Rotation2d.ZERO),
            new GoalEndState(0.0, Rotation2d.ZERO));
    goalPath.name = "Goal";

    // Pretend the robot is at the end of the pathfinding path once pathfinding has started
    Command command = AutoBuilder.pathfindThenFollowPath(goalPath, CONSTRAINTS);
    scheduler.schedule(command);
    step();
    assertEquals(new Translation2d(4.0, 2.0), pathfinder.goal);
    robotPose = new Pose2d(4.0, 2.0, Rotation2d.ZERO);

    stepUntil(() -> !isRunning(command), 2000);
    assertTrue(activePaths.contains("Goal"), "The goal path should be followed: " + activePaths);
    assertEquals("", ActivePathState.getCurrentPathName());
  }
}
