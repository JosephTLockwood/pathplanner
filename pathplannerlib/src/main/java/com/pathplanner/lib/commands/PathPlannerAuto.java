package com.pathplanner.lib.commands;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.AutoFile;
import com.pathplanner.lib.auto.AutoTriggerConditions;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.events.*;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.io.*;
import java.util.*;
import java.util.function.BooleanSupplier;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.event.EventLoop;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.util.UsageReporting;

/** A command that loads and runs an autonomous routine built using PathPlanner. */
public class PathPlannerAuto extends Command {

  /** The name of the path currently being followed, or an empty string if no path is running */
  public static String currentPathName = "";

  private static int instances = 0;

  private Command autoCommand;

  private Pose2d startingPose;

  private final EventLoop autoLoop;

  private final Timer autoTimer;

  private boolean isRunning = false;

  /**
   * Constructs a new PathPlannerAuto command.
   *
   * @param autoName the name of the autonomous routine to load and run
   * @throws AutoBuilderException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine
   */
  public PathPlannerAuto(String autoName) {
    this(autoName, false);
  }

  /**
   * Constructs a new PathPlannerAuto command.
   *
   * @param autoName the name of the autonomous routine to load and run
   * @param mirror Mirror all paths to the other side of the current alliance. For example, if a
   *     path is on the right of the blue alliance side of the field, it will be mirrored to the
   *     left of the blue alliance side of the field.
   * @throws AutoBuilderException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine
   */
  public PathPlannerAuto(String autoName, boolean mirror) {
    if (!AutoBuilder.isConfigured()) {
      throw new AutoBuilderException(
          "AutoBuilder was not configured before attempting to load a PathPlannerAuto from file");
    }

    try {
      initFromAutoFile(AutoFile.fromFile(autoName), mirror);
    } catch (FileNotFoundException e) {
      DriverStationErrors.reportError(e.getMessage(), e.getStackTrace());
      autoCommand = Commands.none();
    } catch (IOException e) {
      DriverStationErrors.reportError(
          "Failed to read file required by auto: " + autoName, e.getStackTrace());
      autoCommand = Commands.none();
    } catch (ParseException e) {
      DriverStationErrors.reportError(
          "Failed to parse JSON in file required by auto: " + autoName, e.getStackTrace());
      autoCommand = Commands.none();
    } catch (FileVersionException e) {
      DriverStationErrors.reportError(
          "Failed to load auto: " + autoName + ". " + e.getMessage(), e.getStackTrace());
      autoCommand = Commands.none();
    }

    addRequirements(autoCommand.getRequirements());
    setName(autoName);
    PPLibTelemetry.registerHotReloadAuto(autoName, this::hotReload);

    this.autoLoop = new EventLoop();
    this.autoTimer = new Timer();

    instances++;
    UsageReporting.reportUsage("PathPlanner/PathPlannerAuto", instances, "");
  }

  /**
   * Create a PathPlannerAuto from a custom command
   *
   * @param autoCommand The command this auto should run
   * @param startingPose The starting pose of the auto. Only used for the getStartingPose method
   */
  public PathPlannerAuto(Command autoCommand, Pose2d startingPose) {
    this.autoCommand = autoCommand;
    this.startingPose = startingPose;

    addRequirements(autoCommand.getRequirements());

    this.autoLoop = new EventLoop();
    this.autoTimer = new Timer();

    instances++;
    UsageReporting.reportUsage("PathPlanner/PathPlannerAuto", instances, "");
  }

  /**
   * Create a PathPlannerAuto from a custom command
   *
   * @param autoCommand The command this auto should run
   */
  public PathPlannerAuto(Command autoCommand) {
    this(autoCommand, Pose2d.ZERO);
  }

  /**
   * Get the starting pose of this auto, relative to a blue alliance origin. If there are no paths
   * in this auto, the starting pose will be null.
   *
   * @return The blue alliance starting pose
   */
  public Pose2d getStartingPose() {
    return startingPose;
  }

  /**
   * Create a trigger that is high when this auto is running, and low when it is not running
   *
   * @return isRunning trigger
   */
  public Trigger isRunning() {
    return condition(() -> isRunning);
  }

  /**
   * Used by path following commands to inform autos of what trajectory is currently being followed
   *
   * @param trajectory The current trajectory being followed
   */
  public static void setCurrentTrajectory(PathPlannerTrajectory trajectory) {
    ActivePathState.setCurrentTrajectory(trajectory);
    if (trajectory == null) {
      currentPathName = "";
    }
  }

  /**
   * Trigger that is high when the given time has elapsed
   *
   * @param time The amount of time this auto should run before the trigger is activated
   * @return timeElapsed trigger
   */
  public Trigger timeElapsed(double time) {
    return condition(() -> autoTimer.hasElapsed(time));
  }

  /**
   * Trigger that is high when the given time has elapsed
   *
   * @param time The amount of time this auto should run before the trigger is activated
   * @return timeElapsed trigger
   */
  public Trigger timeElapsed(Time time) {
    return timeElapsed(time.in(Seconds));
  }

  /**
   * Trigger that is high when within a range of time since the start of this auto
   *
   * @param startTime The starting time of the range
   * @param endTime The ending time of the range
   * @return timeRange trigger
   */
  public Trigger timeRange(double startTime, double endTime) {
    return condition(() -> autoTimer.get() >= startTime && autoTimer.get() <= endTime);
  }

  /**
   * Trigger that is high when within a range of time since the start of this auto
   *
   * @param startTime The starting time of the range
   * @param endTime The ending time of the range
   * @return timeRange trigger
   */
  public Trigger timeRange(Time startTime, Time endTime) {
    return timeRange(startTime.in(Seconds), endTime.in(Seconds));
  }

  /**
   * Create an EventTrigger that will be polled by this auto instead of globally across all path
   * following commands
   *
   * @param eventName The event name that controls this trigger
   * @return EventTrigger for this auto
   */
  public Trigger event(String eventName) {
    return new EventTrigger(autoLoop, eventName);
  }

  /**
   * Create a trigger that will be activated a given time before an event is reached. The trigger
   * will go low after passing the given event.
   *
   * @param eventName The event name
   * @param timeSeconds The amount of time before the event this trigger should activate, in seconds
   * @return beforeEvent trigger
   */
  public Trigger beforeEvent(String eventName, double timeSeconds) {
    return condition(AutoTriggerConditions.beforeEvent(eventName, timeSeconds));
  }

  /**
   * Create a trigger that will be activated a given time before an event is reached. The trigger
   * will go low after passing the given event.
   *
   * @param eventName The event name
   * @param time The amount of time before the event this trigger should activate
   * @return beforeEvent trigger
   */
  public Trigger beforeEvent(String eventName, Time time) {
    return beforeEvent(eventName, time.in(Seconds));
  }

  /**
   * Create a trigger that will be activated when the robot is within a given distance from the
   * start of an event
   *
   * @param eventName The event name
   * @param distanceMeters The distance from the event that will activate this trigger, in meters
   * @return distanceFromEvent trigger
   */
  public Trigger distanceFromEvent(String eventName, double distanceMeters) {
    return condition(
        AutoTriggerConditions.distanceFromEvent(
            AutoBuilder::getCurrentPose, eventName, distanceMeters));
  }

  /**
   * Create a trigger that will be activated when the robot is within a given distance from the
   * start of an event
   *
   * @param eventName The event name
   * @param distance The distance from the event that will activate this trigger
   * @return distanceFromEvent trigger
   */
  public Trigger distanceFromEvent(String eventName, Distance distance) {
    return distanceFromEvent(eventName, distance.in(Meters));
  }

  /**
   * Create a trigger that will be activated when the robot is within a given distance from the end
   * of an event
   *
   * @param eventName The event name
   * @param distanceMeters The distance from the event that will activate this trigger, in meters
   * @return distanceFromEventEnd trigger
   */
  public Trigger distanceFromEventEnd(String eventName, double distanceMeters) {
    return condition(
        AutoTriggerConditions.distanceFromEventEnd(
            AutoBuilder::getCurrentPose, eventName, distanceMeters));
  }

  /**
   * Create a trigger that will be activated when the robot is within a given distance from the end
   * of an event
   *
   * @param eventName The event name
   * @param distance The distance from the event that will activate this trigger
   * @return distanceFromEventEnd trigger
   */
  public Trigger distanceFromEventEnd(String eventName, Distance distance) {
    return distanceFromEventEnd(eventName, distance.in(Meters));
  }

  /**
   * Create a PointTowardsZoneTrigger that will be polled by this auto instead of globally across
   * all path following commands
   *
   * @param zoneName The point towards zone name that controls this trigger
   * @return PointTowardsZoneTrigger for this auto
   */
  public Trigger pointTowardsZone(String zoneName) {
    return new PointTowardsZoneTrigger(autoLoop, zoneName);
  }

  /**
   * Create a trigger that is high when a certain path is being followed
   *
   * @param pathName The name of the path to check for
   * @return activePath trigger
   */
  public Trigger activePath(String pathName) {
    return condition(AutoTriggerConditions.activePath(pathName));
  }

  /**
   * Create a trigger that is high when near a given field position. This field position is not
   * automatically flipped
   *
   * @param fieldPosition The target field position
   * @param toleranceMeters The position tolerance, in meters. The trigger will be high when within
   *     this distance from the target position
   * @return nearFieldPosition trigger
   */
  public Trigger nearFieldPosition(Translation2d fieldPosition, double toleranceMeters) {
    return condition(
        AutoTriggerConditions.nearFieldPosition(
            AutoBuilder::getCurrentPose, fieldPosition, toleranceMeters));
  }

  /**
   * Create a trigger that is high when near a given field position. This field position is not
   * automatically flipped
   *
   * @param fieldPosition The target field position
   * @param tolerance The position tolerance. The trigger will be high when within this distance
   *     from the target position
   * @return nearFieldPosition trigger
   */
  public Trigger nearFieldPosition(Translation2d fieldPosition, Distance tolerance) {
    return nearFieldPosition(fieldPosition, tolerance.in(Meters));
  }

  /**
   * Create a trigger that is high when near a given field position. This field position will be
   * automatically flipped
   *
   * @param blueFieldPosition The target field position if on the blue alliance
   * @param toleranceMeters The position tolerance, in meters. The trigger will be high when within
   *     this distance from the target position
   * @return nearFieldPositionAutoFlipped trigger
   */
  public Trigger nearFieldPositionAutoFlipped(
      Translation2d blueFieldPosition, double toleranceMeters) {
    return condition(
        AutoTriggerConditions.nearFieldPositionAutoFlipped(
            AutoBuilder::getCurrentPose,
            AutoBuilder::shouldFlip,
            blueFieldPosition,
            toleranceMeters));
  }

  /**
   * Create a trigger that is high when near a given field position. This field position will be
   * automatically flipped
   *
   * @param blueFieldPosition The target field position if on the blue alliance
   * @param tolerance The position tolerance. The trigger will be high when within this distance
   *     from the target position
   * @return nearFieldPositionAutoFlipped trigger
   */
  public Trigger nearFieldPositionAutoFlipped(Translation2d blueFieldPosition, Distance tolerance) {
    return nearFieldPositionAutoFlipped(blueFieldPosition, tolerance.in(Meters));
  }

  /**
   * Create a trigger that will be high when the robot is within a given area on the field. These
   * positions will not be automatically flipped
   *
   * @param boundingBoxMin The minimum position of the bounding box for the target field area. The X
   *     and Y coordinates of this position should be less than the max position.
   * @param boundingBoxMax The maximum position of the bounding box for the target field area. The X
   *     and Y coordinates of this position should be greater than the min position.
   * @return inFieldArea trigger
   */
  public Trigger inFieldArea(Translation2d boundingBoxMin, Translation2d boundingBoxMax) {
    return condition(
        AutoTriggerConditions.inFieldArea(
            AutoBuilder::getCurrentPose, boundingBoxMin, boundingBoxMax));
  }

  /**
   * Create a trigger that will be high when the robot is within a given area on the field. These
   * positions will be automatically flipped
   *
   * @param blueBoundingBoxMin The minimum position of the bounding box for the target field area if
   *     on the blue alliance. The X and Y coordinates of this position should be less than the max
   *     position.
   * @param blueBoundingBoxMax The maximum position of the bounding box for the target field area if
   *     on the blue alliance. The X and Y coordinates of this position should be greater than the
   *     min position.
   * @return inFieldAreaAutoFlipped trigger
   */
  public Trigger inFieldAreaAutoFlipped(
      Translation2d blueBoundingBoxMin, Translation2d blueBoundingBoxMax) {
    return condition(
        AutoTriggerConditions.inFieldAreaAutoFlipped(
            AutoBuilder::getCurrentPose,
            AutoBuilder::shouldFlip,
            blueBoundingBoxMin,
            blueBoundingBoxMax));
  }

  /**
   * Create a trigger with a custom condition. This will be polled by this auto's event loop so that
   * its condition is only polled when this auto is running.
   *
   * @param condition The condition represented by this trigger
   * @return Custom condition trigger
   */
  public Trigger condition(BooleanSupplier condition) {
    return new Trigger(autoLoop, condition);
  }

  @Override
  public void initialize() {
    autoCommand.initialize();
    autoTimer.restart();
    isRunning = true;
    autoLoop.poll();
  }

  @Override
  public void execute() {
    autoCommand.execute();
    autoLoop.poll();
  }

  @Override
  public boolean isFinished() {
    return autoCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    autoCommand.end(interrupted);
    autoTimer.stop();
    isRunning = false;
    autoLoop.poll();
  }

  /**
   * Get a list of every path in the given auto (depth first)
   *
   * @param autoName Name of the auto to get the path group from
   * @return List of paths in the auto
   * @throws IOException if attempting to load a file that does not exist or cannot be read
   * @throws ParseException If JSON within file cannot be parsed
   */
  public static List<PathPlannerPath> getPathGroupFromAutoFile(String autoName)
      throws IOException, ParseException {
    return AutoFile.fromFile(autoName).loadPaths();
  }

  /**
   * Reloads the autonomous routine with the given JSON object and updates the requirements of this
   * command.
   *
   * @param autoJson the JSON object representing the updated autonomous routine
   */
  public void hotReload(JSONObject autoJson) {
    try {
      initFromAutoFile(AutoFile.fromJson(autoJson), false);
    } catch (Exception e) {
      DriverStationErrors.reportError("Failed to load path during hot reload", e.getStackTrace());
    }
  }

  private void initFromAutoFile(AutoFile auto, boolean mirror) throws IOException, ParseException {
    Command cmd = CommandUtil.buildCommand(auto.command(), mirror);
    this.startingPose = auto.getStartingPose(AutoBuilder.isHolonomic(), mirror);

    if (auto.resetOdom()) {
      this.autoCommand = Commands.sequence(AutoBuilder.resetOdom(this.startingPose), cmd);
    } else {
      this.autoCommand = cmd;
    }
  }
}
