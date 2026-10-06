package com.pathplanner.lib;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;

/** Robot config and paths shared by the Commands v2 and v3 tests */
public final class TestFixtures {
  private TestFixtures() {}

  public static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          60.0,
          6.0,
          new ModuleConfig(0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
          new Translation2d(0.3, 0.3),
          new Translation2d(0.3, -0.3),
          new Translation2d(-0.3, 0.3),
          new Translation2d(-0.3, -0.3));

  /** A straight 3 meter path along the X axis, with the given event markers */
  public static PathPlannerPath straightPath(String name, EventMarker... markers) {
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
}
