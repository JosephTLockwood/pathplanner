package com.pathplanner.lib.command3;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.AutoFile;
import com.pathplanner.lib.auto.AutoTriggerConditions;
import com.pathplanner.lib.events.EventConditions;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.util.UsageReporting;

/**
 * A Commands v3 command that loads and runs an autonomous routine built using PathPlanner.
 *
 * <p>The triggers created by this auto are polled by the scheduler, and are only active while this
 * auto is running. Commands bound to them are scheduled independently of the auto.
 */
public class PathPlannerAuto implements Command {
  private static int instances = 0;

  private final Scheduler scheduler;
  private final String name;
  private final Timer autoTimer = new Timer();

  private Command autoCommand;
  private Pose2d startingPose;
  private int stopCount = 0;

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
    this(Scheduler.getDefault(), autoName, mirror);
  }

  /**
   * Constructs a new PathPlannerAuto command.
   *
   * @param scheduler The scheduler that will poll this auto's triggers
   * @param autoName the name of the autonomous routine to load and run
   * @param mirror Mirror all paths to the other side of the current alliance. For example, if a
   *     path is on the right of the blue alliance side of the field, it will be mirrored to the
   *     left of the blue alliance side of the field.
   * @throws AutoBuilderException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine
   */
  public PathPlannerAuto(Scheduler scheduler, String autoName, boolean mirror) {
    if (!AutoBuilder.isConfigured()) {
      throw new AutoBuilderException(
          "AutoBuilder was not configured before attempting to load a PathPlannerAuto from file");
    }

    this.scheduler = scheduler;
    this.name = autoName;

    try {
      initFromAutoFile(AutoFile.fromFile(autoName), mirror);
    } catch (FileNotFoundException e) {
      DriverStationErrors.reportError(e.getMessage(), e.getStackTrace());
      autoCommand = CommandUtil.none();
    } catch (IOException e) {
      DriverStationErrors.reportError(
          "Failed to read file required by auto: " + autoName, e.getStackTrace());
      autoCommand = CommandUtil.none();
    } catch (ParseException e) {
      DriverStationErrors.reportError(
          "Failed to parse JSON in file required by auto: " + autoName, e.getStackTrace());
      autoCommand = CommandUtil.none();
    } catch (FileVersionException e) {
      DriverStationErrors.reportError(
          "Failed to load auto: " + autoName + ". " + e.getMessage(), e.getStackTrace());
      autoCommand = CommandUtil.none();
    }

    PPLibTelemetry.registerHotReloadAuto(autoName, this::hotReload);

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
    this(Scheduler.getDefault(), autoCommand, startingPose);
  }

  /**
   * Create a PathPlannerAuto from a custom command
   *
   * @param scheduler The scheduler that will poll this auto's triggers
   * @param autoCommand The command this auto should run
   * @param startingPose The starting pose of the auto. Only used for the getStartingPose method
   */
  public PathPlannerAuto(Scheduler scheduler, Command autoCommand, Pose2d startingPose) {
    this.scheduler = scheduler;
    this.name = autoCommand.name();
    this.autoCommand = autoCommand;
    this.startingPose = startingPose;

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

  @Override
  public void run(Coroutine coroutine) {
    autoTimer.restart();

    coroutine.await(autoCommand);

    stopRunning();
  }

  @Override
  public void onCancel() {
    stopRunning();
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return autoCommand.requirements();
  }

  @Override
  public int priority() {
    return autoCommand.priority();
  }

  @Override
  public String toString() {
    return name();
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
   * Create a trigger with a custom condition. This will be polled by the scheduler, and is only
   * active while this auto is running.
   *
   * @param condition The condition represented by this trigger
   * @return Custom condition trigger
   */
  public Trigger condition(BooleanSupplier condition) {
    return new Trigger(scheduler, whileRunning(condition));
  }

  /**
   * Create a trigger that is high when this auto is running, and low when it is not running
   *
   * @return isRunning trigger
   */
  public Trigger isRunning() {
    return new Trigger(scheduler, autoTimer::isRunning);
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
   * Create an event trigger that is only active while this auto is running
   *
   * @param eventName The event name that controls this trigger
   * @return Event trigger for this auto
   */
  public Trigger event(String eventName) {
    return condition(EventTrigger.pollCondition(eventName));
  }

  /**
   * Trigger that is high when there is less than the given time before the given event will be
   * reached
   *
   * @param eventName The name of the event
   * @param timeSeconds The time before the event, in seconds
   * @return beforeEvent trigger
   */
  public Trigger beforeEvent(String eventName, double timeSeconds) {
    return condition(AutoTriggerConditions.beforeEvent(eventName, timeSeconds));
  }

  /**
   * Trigger that is high when there is less than the given time before the given event will be
   * reached
   *
   * @param eventName The name of the event
   * @param time The time before the event
   * @return beforeEvent trigger
   */
  public Trigger beforeEvent(String eventName, Time time) {
    return beforeEvent(eventName, time.in(Seconds));
  }

  /**
   * Trigger that is high when the robot is within a distance of the start of the given event
   *
   * @param eventName The name of the event
   * @param distanceMeters The distance from the event that will activate this trigger, in meters
   * @return distanceFromEvent trigger
   */
  public Trigger distanceFromEvent(String eventName, double distanceMeters) {
    return condition(
        AutoTriggerConditions.distanceFromEvent(
            AutoBuilder::getCurrentPose, eventName, distanceMeters));
  }

  /**
   * Trigger that is high when the robot is within a distance of the start of the given event
   *
   * @param eventName The name of the event
   * @param distance The distance from the event that will activate this trigger
   * @return distanceFromEvent trigger
   */
  public Trigger distanceFromEvent(String eventName, Distance distance) {
    return distanceFromEvent(eventName, distance.in(Meters));
  }

  /**
   * Trigger that is high when the robot is within a distance of the end of the given event
   *
   * @param eventName The name of the event
   * @param distanceMeters The distance from the event end that will activate this trigger, in
   *     meters
   * @return distanceFromEventEnd trigger
   */
  public Trigger distanceFromEventEnd(String eventName, double distanceMeters) {
    return condition(
        AutoTriggerConditions.distanceFromEventEnd(
            AutoBuilder::getCurrentPose, eventName, distanceMeters));
  }

  /**
   * Trigger that is high when the robot is within a distance of the end of the given event
   *
   * @param eventName The name of the event
   * @param distance The distance from the event end that will activate this trigger
   * @return distanceFromEventEnd trigger
   */
  public Trigger distanceFromEventEnd(String eventName, Distance distance) {
    return distanceFromEventEnd(eventName, distance.in(Meters));
  }

  /**
   * Create a point towards zone trigger that is only active while this auto is running
   *
   * @param zoneName The point towards zone name that controls this trigger
   * @return Point towards zone trigger for this auto
   */
  public Trigger pointTowardsZone(String zoneName) {
    return condition(() -> EventConditions.isWithinZone(zoneName));
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
   * Reloads the autonomous routine with the given JSON object. The requirements of this command
   * will be updated the next time it is scheduled.
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
    Command command = CommandUtil.buildCommand(auto.command(), mirror);
    this.startingPose = auto.getStartingPose(AutoBuilder.isHolonomic(), mirror);

    if (auto.resetOdom()) {
      this.autoCommand =
          Command.sequence(AutoBuilder.resetOdom(this.startingPose), command).named(name);
    } else {
      this.autoCommand = command;
    }
  }

  private void stopRunning() {
    autoTimer.stop();
    stopCount++;
  }

  /**
   * Wrap a condition so it is only true while this auto is running. The condition is also checked
   * on the first poll after the auto stops, so events that happen in the auto's final loop are not
   * missed.
   */
  private BooleanSupplier whileRunning(BooleanSupplier condition) {
    return new BooleanSupplier() {
      private int lastStopCount = stopCount;

      @Override
      public boolean getAsBoolean() {
        // Always poll the condition so that it stays up to date while the auto is not running
        boolean value = condition.getAsBoolean();
        boolean active = autoTimer.isRunning() || lastStopCount != stopCount;
        lastStopCount = stopCount;

        return active && value;
      }
    };
  }
}
