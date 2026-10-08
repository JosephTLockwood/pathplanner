package com.pathplanner.lib.path2;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

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

  @Test
  void loadsStraightPath() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Straight");
    assertEquals("Straight", path.name);
    assertEquals(2, path.getNodes().size());
    assertEquals(1, path.getBranches().size());

    PathGraph.Node start = path.getStartNode();
    assertEquals(new Translation2d(-6.0, 3.0), start.waypoint().position());
    assertEquals(0.0, start.waypoint().rotation().getRadians(), EPSILON);
    assertEquals(2.0, start.waypoint().maxVelocityMPS(), EPSILON);
    assertEquals(Math.toRadians(270.0), start.waypoint().maxAngularVelocityRadPerSec(), EPSILON);
    assertEquals(
        Math.toRadians(360.0), start.waypoint().maxAngularAccelerationRadPerSecSq(), EPSILON);

    PathGraph.Branch branch = path.getOutgoingBranches(start.id()).get(0);
    assertEquals(new PathGraph.DistanceTransition(0.25), branch.transition());
    PathGraph.Node end = path.getNode(branch.targetId());
    assertEquals(new Translation2d(-4.0, 2.0), end.waypoint().position());
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

    // A pose waypoint's heading is saved in radians
    PathGraph.Node middle = path.getNode(first.targetId());
    assertTrue(middle.waypoint().isPose());
    assertEquals(new Translation2d(-3.0, 1.5), middle.waypoint().position());
    assertEquals(Math.PI / 2.0, middle.waypoint().rotation().getRadians(), EPSILON);
    assertEquals(List.of("Score"), middle.waypoint().events());

    PathGraph.Branch second = path.getOutgoingBranches(middle.id()).get(0);
    assertEquals(new PathGraph.DistanceTransition(0.5), second.transition());
    PathGraph.Node end = path.getNode(second.targetId());
    assertFalse(end.waypoint().isPose());
    assertFalse(end.waypoint().isPointTowards());
    assertEquals(new Translation2d(-2.0, 0.0), end.waypoint().position());
  }

  @Test
  void loadsLeaveStart() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Leave Start");
    PathGraph.Node start = path.getStartNode();
    assertEquals(new Translation2d(7.25, 2.0), start.waypoint().position());
    assertEquals(Math.PI, Math.abs(start.waypoint().rotation().getRadians()), EPSILON);

    PathGraph.Node end = path.getNode(path.getOutgoingBranches(start.id()).get(0).targetId());
    assertEquals(new Translation2d(5.25, 2.0), end.waypoint().position());
    assertEquals(-135.0, end.waypoint().rotation().getDegrees(), 1e-9);
  }

  @Test
  void loadsAutoWithCommandAndStartingPose() throws Exception {
    assertTrue(AutoGraph.isGraphFile("Fixture Auto"));
    AutoGraph auto = AutoGraph.fromFile("Fixture Auto");
    assertEquals(2, auto.nodes().size());
    assertTrue(auto.startingPoseInitialized());
    assertEquals(new Pose2d(-6.0, 3.0, Rotation2d.ZERO), auto.startingPose());

    AutoGraph.Node start = auto.getStartNode();
    assertTrue(start.isPath());
    assertEquals("Straight", start.pathName());
    assertEquals(List.of("Spinup"), start.events());

    AutoGraph.Branch branch = auto.getOutgoingBranches(start.id()).get(0);
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
    assertEquals(new Translation2d(7.25, 2.0), auto.startingPose().getTranslation());
    assertEquals(Math.PI, Math.abs(auto.startingPose().getRotation().getRadians()), EPSILON);
  }

  @Test
  void legacyFilesStillLoad() throws Exception {
    PathPlannerPath legacy = PathPlannerPath.fromPathFile("Start to Shoot");
    assertNotNull(legacy);
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
  void flipsAndMirrorsAboutTheFieldCenter() throws Exception {
    PathGraph path = PathGraph.fromPathFile("Leave Start");
    PathGraph.Node flipped = path.flip().getStartNode();
    // The default symmetry is rotational: the other alliance's start is the same pose turned
    // half a turn about the center of the field
    assertEquals(new Translation2d(-7.25, -2.0), flipped.waypoint().position());
    assertEquals(0.0, flipped.waypoint().rotation().getRadians(), EPSILON);

    PathGraph.Node mirrored = path.mirror().getStartNode();
    assertEquals(new Translation2d(7.25, -2.0), mirrored.waypoint().position());
  }
}
