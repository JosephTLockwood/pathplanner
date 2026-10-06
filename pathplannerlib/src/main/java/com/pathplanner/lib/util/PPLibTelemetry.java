package com.pathplanner.lib.util;

import com.pathplanner.lib.path.PathPlannerPath;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.function.Consumer;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.*;
import org.wpilib.system.Filesystem;

/** Utility class for sending data to the PathPlanner app via NT4 */
public class PPLibTelemetry {

  private static boolean compMode = false;

  private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  private static final DoubleArrayPublisher velPub =
      nt.getDoubleArrayTopic("/PathPlanner/vel").publish();

  private static final StructPublisher<Pose2d> posePub =
      nt.getStructTopic("/PathPlanner/currentPose", Pose2d.struct).publish();

  private static final StructArrayPublisher<Pose2d> pathPub =
      nt.getStructArrayTopic("/PathPlanner/activePath", Pose2d.struct).publish();

  private static final StructPublisher<Pose2d> targetPosePub =
      nt.getStructTopic("/PathPlanner/targetPose", Pose2d.struct).publish();

  private static final Map<String, List<PathPlannerPath>> hotReloadPaths = new HashMap<>();

  private static final Map<String, List<Consumer<JSONObject>>> hotReloadAutos = new HashMap<>();

  private static NetworkTableListener hotReloadPathListener = null;

  private static NetworkTableListener hotReloadAutoListener = null;

  /** Enable competition mode. This will disable hot reload. */
  public static void enableCompetitionMode() {
    compMode = true;
  }

  /**
   * Set the path following actual/target velocities
   *
   * @param actualVel Actual robot velocity in m/s
   * @param commandedVel Target robot velocity in m/s
   * @param actualAngVel Actual angular velocity in rad/s
   * @param commandedAngVel Target angular velocity in rad/s
   */
  public static void setVelocities(
      double actualVel, double commandedVel, double actualAngVel, double commandedAngVel) {
    if (!compMode) {
      velPub.set(new double[] {actualVel, commandedVel, actualAngVel, commandedAngVel});
    }
  }

  /**
   * Set the current robot pose
   *
   * @param pose Current robot pose
   */
  public static void setCurrentPose(Pose2d pose) {
    if (!compMode) {
      posePub.set(pose);
    }
  }

  /**
   * Set the current path being followed
   *
   * @param path The current path
   */
  public static void setCurrentPath(PathPlannerPath path) {
    if (!compMode) {
      // Use poses for simplicity
      pathPub.set(path.getPathPoses().toArray(new Pose2d[0]));
    }
  }

  /**
   * Set the target robot pose
   *
   * @param targetPose Target robot pose
   */
  public static void setTargetPose(Pose2d targetPose) {
    if (!compMode) {
      targetPosePub.set(targetPose);
    }
  }

  /**
   * Register a path for hot reload. This is used internally.
   *
   * @param pathName Name of the path
   * @param path Reference to the path
   */
  public static void registerHotReloadPath(String pathName, PathPlannerPath path) {
    if (!compMode) {
      ensureHotReloadListenersInitialized();
      if (!hotReloadPaths.containsKey(pathName)) {
        hotReloadPaths.put(pathName, new ArrayList<>());
      }
      hotReloadPaths.get(pathName).add(path);
    }
  }

  /**
   * Register an auto for hot reload. This is used internally.
   *
   * @param autoName Name of the auto
   * @param hotReload Function that will reload the auto from the updated auto json
   */
  public static void registerHotReloadAuto(String autoName, Consumer<JSONObject> hotReload) {
    if (!compMode) {
      ensureHotReloadListenersInitialized();
      if (!hotReloadAutos.containsKey(autoName)) {
        hotReloadAutos.put(autoName, new ArrayList<>());
      }
      hotReloadAutos.get(autoName).add(hotReload);
    }
  }

  private static void ensureHotReloadListenersInitialized() {
    if (hotReloadPathListener == null) {
      hotReloadPathListener =
          NetworkTableListener.createListener(
              nt.getStringTopic("/PathPlanner/HotReload/hotReloadPath"),
              EnumSet.of(NetworkTableEvent.Kind.VALUE_REMOTE),
              PPLibTelemetry::handlePathHotReloadEvent);
    }
    if (hotReloadAutoListener == null) {
      hotReloadAutoListener =
          NetworkTableListener.createListener(
              nt.getStringTopic("/PathPlanner/HotReload/hotReloadAuto"),
              EnumSet.of(NetworkTableEvent.Kind.VALUE_REMOTE),
              PPLibTelemetry::handleAutoHotReloadEvent);
    }
  }

  private static void handlePathHotReloadEvent(NetworkTableEvent event) {
    if (!compMode) {
      if (RobotState.isEnabled()) {
        DriverStationErrors.reportWarning("Ignoring path hot reload, robot is enabled", false);
        return;
      }
      try {
        String jsonStr = event.valueData.value.getString();
        JSONObject json = (JSONObject) new JSONParser().parse(jsonStr);
        String name = (String) json.get("name");
        JSONObject pathJson = (JSONObject) json.get("path");
        if (hotReloadPaths.containsKey(name)) {
          for (PathPlannerPath path : hotReloadPaths.get(name)) {
            path.hotReload(pathJson);
          }
        }
        if (RobotBase.isReal()) {
          File pathFile =
              new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + name + ".path");
          try (FileWriter writer = new FileWriter(pathFile)) {
            writer.write(pathJson.toJSONString());
            writer.flush();
          } catch (IOException _) {
            DriverStationErrors.reportWarning(
                "Failed to save updated path file contents, please re-deploy code", false);
          }
        }
      } catch (Exception _) {
        // Ignore
      }
    }
  }

  private static void handleAutoHotReloadEvent(NetworkTableEvent event) {
    if (!compMode) {
      if (RobotState.isEnabled()) {
        DriverStationErrors.reportWarning("Ignoring auto hot reload, robot is enabled", false);
        return;
      }
      try {
        String jsonStr = event.valueData.value.getString();
        JSONObject json = (JSONObject) new JSONParser().parse(jsonStr);
        String name = (String) json.get("name");
        JSONObject autoJson = (JSONObject) json.get("auto");
        if (hotReloadAutos.containsKey(name)) {
          for (Consumer<JSONObject> hotReload : hotReloadAutos.get(name)) {
            hotReload.accept(autoJson);
          }
        }
        if (RobotBase.isReal()) {
          File pathFile =
              new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + name + ".auto");
          try (FileWriter writer = new FileWriter(pathFile)) {
            writer.write(autoJson.toJSONString());
            writer.flush();
          } catch (IOException _) {
            DriverStationErrors.reportWarning(
                "Failed to save updated auto file contents, please re-deploy code", false);
          }
        }
      } catch (Exception _) {
        // Ignore
      }
    }
  }
}
