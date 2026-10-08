package com.pathplanner.lib.path2;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Loads files saved by the 2027 app. Every expected pose is in WPILib's blue alliance origin on the
 * default 16.54 m by 8.07 m field, converted from what the app saved with blueX = 8.27 - x, blueY =
 * 4.035 - y, and heading + 180 degrees.
 */
class GraphFileTest {
  private static final double EPSILON = 1e-9;

  private FixtureProject project;

  @BeforeEach
  void openProject() {
    project = FixtureProject.open2027();
  }

  @AfterEach
  void closeProject() {
    project.close();
  }

  static void assertPose(Pose2d expected, Translation2d position, Rotation2d rotation) {
    assertEquals(expected.getX(), position.getX(), 1e-9, "x");
    assertEquals(expected.getY(), position.getY(), 1e-9, "y");
    if (rotation != null) {
      assertEquals(
          0.0, rotation.minus(expected.getRotation()).getRadians(), 1e-9, "heading " + rotation);
    }
  }

  @Test
  void versionChecks() {
    assertTrue(FileVersion.isGraphFormat("2027.1"));
    assertTrue(FileVersion.isGraphFormat("2027.2"));
    assertTrue(FileVersion.isGraphFormat("2028.0"));
    assertFalse(FileVersion.isGraphFormat("2027.0"));
    assertFalse(FileVersion.isGraphFormat("2025.0"));
    assertFalse(FileVersion.isGraphFormat("not a version"));
    assertTrue(FileVersion.isLegacyFormat("2025.0"));
    assertFalse(FileVersion.isLegacyFormat("2027.1"));
  }

  /**
   * Follows a point through the app's own drawing code, independently of {@link AppCoordinates},
   * and checks that it lands where the conversion puts it.
   *
   * <p>The 2027 app draws images/field26.png (3508 x 1814 px, 200 px/m) turned half a turn
   * (field_image.dart: rotate180 = true, RotatedBox(quarterTurns: 2)), and draws a saved point at
   * (width / 2 + x * 200, height / 2 - y * 200) on screen (path_painter_util.dart
   * pointToPixelOffset). Undoing the half turn gives the pixel on the image file. In the image file
   * the blue alliance is the left half, and the field outline is inset 0.5 m on every side, which
   * is how the 2026 app drew it in the blue alliance origin (marginMeters = 0.5).
   */
  @Test
  void appDrawingMatchesTheConversion() throws Exception {
    double widthPx = 3508;
    double heightPx = 1814;
    double pxPerMeter = 200;
    double margin = 0.5;

    // "Leave Start" starts at (7.25, 2.0) in the file
    double screenX = widthPx / 2 + 7.25 * pxPerMeter;
    double screenY = heightPx / 2 - 2.0 * pxPerMeter;
    double imageX = widthPx - screenX;
    double imageY = heightPx - screenY;
    assertTrue(imageX < widthPx / 2, "Leave Start should be drawn on the blue half of the field");

    // The 2026 app's mapping from the image file to the blue origin
    double blueX = imageX / pxPerMeter - margin;
    double blueY = (heightPx - imageY) / pxPerMeter - margin;

    Translation2d loaded =
        PathGraph.fromPathFile("Leave Start").getStartNode().waypoint().position();
    assertEquals(blueX, loaded.getX(), 1e-9);
    assertEquals(blueY, loaded.getY(), 1e-9);
  }

  @Test
  void startNearTheBlueWallLoadsWithASmallBlueX() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Leave Start");
    PathGraph.Node start = path.getStartNode();
    // 1.02 m out from the blue alliance wall, facing down the field, away from it
    assertPose(
        new Pose2d(1.02, 2.035, Rotation2d.ZERO),
        start.waypoint().position(),
        start.waypoint().rotation());

    PathGraph.Node end = path.getNode(path.getOutgoingBranches(start.id()).get(0).targetId());
    assertPose(
        new Pose2d(3.02, 2.035, Rotation2d.fromDegrees(45.0)),
        end.waypoint().position(),
        end.waypoint().rotation());
  }

  @Test
  void loadsStraightPath() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Straight");
    assertEquals("Straight", path.name);
    assertEquals(2, path.getNodes().size());
    assertEquals(1, path.getBranches().size());

    PathGraph.Node start = path.getStartNode();
    assertPose(
        new Pose2d(14.27, 1.035, Rotation2d.PI),
        start.waypoint().position(),
        start.waypoint().rotation());
    assertEquals(2.0, start.waypoint().maxVelocityMPS(), EPSILON);
    assertEquals(Math.toRadians(270.0), start.waypoint().maxAngularVelocityRadPerSec(), EPSILON);
    assertEquals(
        Math.toRadians(360.0), start.waypoint().maxAngularAccelerationRadPerSecSq(), EPSILON);

    PathGraph.Branch branch = path.getOutgoingBranches(start.id()).get(0);
    assertEquals(new PathGraph.DistanceTransition(0.25), branch.transition());
    PathGraph.Node end = path.getNode(branch.targetId());
    assertPose(
        new Pose2d(12.27, 2.035, Rotation2d.PI),
        end.waypoint().position(),
        end.waypoint().rotation());
    assertEquals(0.1, end.endToleranceMeters(), EPSILON);
    assertEquals(Math.toRadians(2.0), end.endToleranceRadians(), EPSILON);
    assertTrue(path.getOutgoingBranches(end.id()).isEmpty());
  }

  @Test
  void loadsHeadingWaypointsAndEvents() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Rotate Events");
    PathGraph.Node start = path.getStartNode();

    PathGraph.Branch first = path.getOutgoingBranches(start.id()).get(0);
    assertEquals(new PathGraph.DistanceTransition(0.25), first.transition());
    assertEquals(List.of(new PathGraph.Event("Intake", 0.5)), first.events());

    // The app saved this heading as 90 degrees, in radians
    PathGraph.Node middle = path.getNode(first.targetId());
    assertTrue(middle.waypoint().isPose());
    assertPose(
        new Pose2d(11.27, 2.535, Rotation2d.fromDegrees(-90.0)),
        middle.waypoint().position(),
        middle.waypoint().rotation());
    assertEquals(List.of("Score"), middle.waypoint().events());

    PathGraph.Branch second = path.getOutgoingBranches(middle.id()).get(0);
    assertEquals(new PathGraph.DistanceTransition(0.5), second.transition());
    PathGraph.Node end = path.getNode(second.targetId());
    assertFalse(end.waypoint().isPose());
    assertFalse(end.waypoint().isPointTowards());
    assertPose(new Pose2d(10.27, 4.035, Rotation2d.ZERO), end.waypoint().position(), null);
  }

  @Test
  void loadsAutoWithCommandAndStartingPose() throws Exception {
    assertTrue(AutoGraph.isGraphFile("Fixture Auto"));
    AutoGraph auto = AutoGraph.fromFile("Fixture Auto");
    assertEquals(2, auto.nodes().size());
    assertTrue(auto.startingPoseInitialized());
    Pose2d start = auto.startingPose();
    assertPose(
        new Pose2d(14.27, 1.035, Rotation2d.PI), start.getTranslation(), start.getRotation());

    AutoGraph.Node first = auto.getStartNode();
    assertTrue(first.isPath());
    assertEquals("Straight", first.pathName());
    assertEquals(List.of("Spinup"), first.events());

    AutoGraph.Branch branch = auto.getOutgoingBranches(first.id()).get(0);
    assertFalse(branch.isCondition());
    AutoGraph.Node command = auto.getNode(branch.targetId());
    assertFalse(command.isPath());
    assertEquals("PrintHello", command.commandName());
    assertEquals(List.of("Straight"), auto.getPathNames());
  }

  @Test
  void leaveStartAutoStartsWhereThePathStarts() throws Exception {
    AutoGraph auto = AutoGraph.fromFile("Leave Start");
    assertTrue(auto.startingPoseInitialized());
    PathGraph.Node pathStart = PathGraph.fromPathFile("Leave Start").getStartNode();
    assertPose(
        new Pose2d(pathStart.waypoint().position(), pathStart.waypoint().rotation()),
        auto.startingPose().getTranslation(),
        auto.startingPose().getRotation());
  }

  @Test
  void legacyFilesStillLoad() throws Exception {
    // 2025.0 files are already in the blue alliance origin and are not converted
    PathPlannerPath legacy = PathPlannerPath.fromPathFile("Start to Shoot");
    assertEquals(new Translation2d(3.4, 5.7), legacy.getPoint(0).position);
    assertEquals(new Translation2d(2.4, 5.9), legacy.getPoint(legacy.numPoints() - 1).position);
    assertEquals(Rotation2d.ZERO, legacy.getIdealStartingState().rotation());
    assertEquals(Rotation2d.fromDegrees(-40.0), legacy.getGoalEndState().rotation());
    assertFalse(AutoGraph.isGraphFile("Shoot and Leave"));
    assertFalse(PathGraph.isGraphFile("Start to Shoot"));
  }

  @Test
  void eachLoaderRejectsTheOtherFormat() {
    FileVersionException graphAsLegacy =
        assertThrows(FileVersionException.class, () -> PathPlannerPath.fromPathFile("Straight"));
    assertTrue(graphAsLegacy.getMessage().contains("PathGraph.fromPathFile"));
    assertThrows(FileVersionException.class, () -> PathGraph.fromPathFile("Start to Shoot"));
    assertThrows(FileVersionException.class, () -> AutoGraph.fromFile("Shoot and Leave"));
  }

  @Test
  void aFieldSizeThatDisagreesWithTheNavgridFailsToLoad() {
    double length = FlippingUtil.fieldSizeX;
    try {
      FlippingUtil.fieldSizeX = 17.548;
      IllegalStateException error =
          assertThrows(IllegalStateException.class, () -> PathGraph.fromPathFile("Straight"));
      assertTrue(error.getMessage().contains("navgrid.json"));
      assertThrows(IllegalStateException.class, () -> AutoGraph.fromFile("Leave Start"));
    } finally {
      FlippingUtil.fieldSizeX = length;
    }
  }

  @Test
  void settingsSavedByTheNewAppLoad() throws Exception {
    // The 2027 app stopped saving maxDriveSpeed. It shows the speed where motor torque at 12 V
    // balances friction torque, 4.7 m/s for this Kraken X60 at 7.364:1 on a 0.055 m wheel.
    RobotConfig config = RobotConfig.fromGUISettings();
    double kv = (6000.0 * 2.0 * Math.PI / 60.0) / (12.0 - 12.0 / 366.0 * 2.0) / 7.364;
    assertEquals(12.0 * kv * 0.055, config.moduleConfig.maxDriveVelocityMPS, 1e-9);
    assertEquals(4.72, config.moduleConfig.maxDriveVelocityMPS, 0.01);
    assertEquals(0.0, config.moduleConfig.torqueLoss, 1e-9);
  }

  @Test
  void settingsSavedByTheOldAppStillLoad() throws Exception {
    project.close();
    project = FixtureProject.open2025();
    RobotConfig config = RobotConfig.fromGUISettings();
    assertEquals(4.54, config.moduleConfig.maxDriveVelocityMPS, 1e-9);
  }

  @Test
  void flipsForRedLikeA2025PathThroughTheSamePoses() throws Exception {
    PathGraph graph = PathGraph.fromPathFile("Leave Start");
    GraphWaypoint start = graph.getStartNode().waypoint();
    GraphWaypoint end =
        graph
            .getNode(graph.getOutgoingBranches(graph.getStartNode().id()).get(0).targetId())
            .waypoint();

    // The same drive drawn in the 2026 app's format, then flipped as 2025 paths are flipped
    PathPlannerPath legacy =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(start.position(), Rotation2d.ZERO),
                new Pose2d(end.position(), Rotation2d.ZERO)),
            new PathConstraints(2.0, 2.0, 4.0, 4.0),
            new IdealStartingState(0.0, start.rotation()),
            new GoalEndState(0.0, end.rotation()));
    PathPlannerPath legacyRed = legacy.flipPath();

    PathGraph red = graph.flip();
    GraphWaypoint redStart = red.getStartNode().waypoint();
    GraphWaypoint redEnd =
        red.getNode(red.getOutgoingBranches(red.getStartNode().id()).get(0).targetId()).waypoint();

    assertPose(
        new Pose2d(legacyRed.getPoint(0).position, legacyRed.getIdealStartingState().rotation()),
        redStart.position(),
        redStart.rotation());
    assertPose(
        new Pose2d(
            legacyRed.getPoint(legacyRed.numPoints() - 1).position,
            legacyRed.getGoalEndState().rotation()),
        redEnd.position(),
        redEnd.rotation());
    // Rotational symmetry: 1.02 m from the red wall, facing away from it
    assertPose(new Pose2d(15.52, 6.035, Rotation2d.PI), redStart.position(), redStart.rotation());
  }

  @Test
  void mirrorsLikeA2025Path() throws Exception {
    PathGraph.Node mirrored = PathGraph.fromPathFile("Leave Start").mirror().getStartNode();
    assertPose(
        new Pose2d(1.02, 8.07 - 2.035, Rotation2d.ZERO),
        mirrored.waypoint().position(),
        mirrored.waypoint().rotation());
  }
}
