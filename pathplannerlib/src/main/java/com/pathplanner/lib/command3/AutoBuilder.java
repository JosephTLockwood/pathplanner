package com.pathplanner.lib.command3;

import static org.wpilib.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.AutoFile;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path2.PathGraph;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PPLibTesting;
import java.util.ArrayList;
import java.util.List;
import java.util.function.*;
import java.util.stream.Stream;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.Selectable;
import org.wpilib.units.measure.LinearVelocity;

/** Utility class used to build Commands v3 auto routines */
public class AutoBuilder {
  private static Globals globals = new Globals();

  static {
    PPLibTesting.addResetHook(AutoBuilder::resetForTesting);
  }

  private AutoBuilder() {}

  /**
   * Configures the AutoBuilder for using PathPlanner's built-in commands.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
   *     speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow paths
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
   *     the field. This will maintain a global blue alliance origin.
   * @param driveRequirements the mechanisms that make up the robot's drive train
   */
  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Mechanism... driveRequirements) {
    configureCustom(
        path ->
            new FollowPathCommand(
                path,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlipPath,
                driveRequirements),
        poseSupplier,
        resetPose,
        shouldFlipPath,
        robotConfig.isHolonomic);

    if (robotConfig.isHolonomic) {
      globals.pathGraphCommandBuilder =
          path ->
              new FollowPathGraphCommand(
                  path,
                  poseSupplier,
                  robotRelativeSpeedsSupplier,
                  output,
                  robotConfig,
                  shouldFlipPath,
                  driveRequirements);
    }
    globals.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel) ->
            new PathfindingCommand(
                pose,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                driveRequirements);
    globals.pathfindThenFollowPathCommandBuilder =
        (path, constraints) ->
            new PathfindThenFollowPath(
                path,
                constraints,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlipPath,
                driveRequirements);
    globals.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder for using PathPlanner's built-in commands.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
   *     speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds.
   * @param controller Path following controller that will be used to follow paths
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
   *     the field. This will maintain a global blue alliance origin.
   * @param driveRequirements the mechanisms that make up the robot's drive train
   */
  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
      Consumer<ChassisVelocities> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Mechanism... driveRequirements) {
    configure(
        poseSupplier,
        resetPose,
        robotRelativeSpeedsSupplier,
        (speeds, _) -> output.accept(speeds),
        controller,
        robotConfig,
        shouldFlipPath,
        driveRequirements);
  }

  /**
   * Configures the AutoBuilder with custom path following command builder. Building pathfinding
   * commands is not supported if using a custom command builder.
   *
   * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param shouldFlipPose Supplier that determines if the starting pose should be flipped to the
   *     other side of the field. This will maintain a global blue alliance origin. NOTE: paths will
   *     not be flipped when configured with a custom path following command. Flipping the paths
   *     must be handled in your command.
   * @param isHolonomic Does the robot have a holonomic drivetrain
   */
  public static void configureCustom(
      Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      BooleanSupplier shouldFlipPose,
      boolean isHolonomic) {
    if (globals.configured) {
      DriverStationErrors.reportError(
          "Auto builder has already been configured. This is likely in error.", true);
    }

    globals.pathFollowingCommandBuilder = pathFollowingCommandBuilder;
    globals.pathGraphCommandBuilder = null;
    globals.poseSupplier = poseSupplier;
    globals.resetPose = resetPose;
    globals.configured = true;
    globals.shouldFlipPath = shouldFlipPose;
    globals.isHolonomic = isHolonomic;
    globals.pathfindingConfigured = false;
  }

  /**
   * Configures the AutoBuilder with custom path following command builder. Building pathfinding
   * commands is not supported if using a custom command builder.
   *
   * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param isHolonomic Does the robot have a holonomic drivetrain
   */
  public static void configureCustom(
      Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      boolean isHolonomic) {
    configureCustom(pathFollowingCommandBuilder, poseSupplier, resetPose, () -> false, isHolonomic);
  }

  /**
   * Returns whether the AutoBuilder has been configured.
   *
   * @return true if the AutoBuilder has been configured, false otherwise
   */
  public static boolean isConfigured() {
    return globals.configured;
  }

  /**
   * Returns whether the AutoBuilder has been configured for pathfinding.
   *
   * @return true if the AutoBuilder has been configured for pathfinding, false otherwise
   */
  public static boolean isPathfindingConfigured() {
    return globals.pathfindingConfigured;
  }

  /**
   * Resets all static state to the values set at class initialization time.
   *
   * <p>This method should not be called during a competition. It is intended to be called by
   * PPLibTesting.resetForTesting().
   */
  public static void resetForTesting() {
    globals = new Globals();
  }

  /**
   * Get the current robot pose
   *
   * @return Current robot pose
   */
  public static Pose2d getCurrentPose() {
    return globals.poseSupplier.get();
  }

  /**
   * Get if a path or field position should currently be flipped
   *
   * @return True if path/positions should be flipped
   */
  public static boolean shouldFlip() {
    return globals.shouldFlipPath.getAsBoolean();
  }

  /**
   * Builds a command to follow a path. PathPlannerLib commands will also trigger event markers
   * along the way.
   *
   * @param path the path to follow
   * @return a path following command with for the given path
   * @throws AutoBuilderException if the AutoBuilder has not been configured
   */
  public static Command followPath(PathPlannerPath path) {
    if (!isConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a path following command before being configured");
    }

    return globals.pathFollowingCommandBuilder.apply(path);
  }

  /**
   * Builds a command to follow a path drawn in the PathPlanner 2027 app. The command signals the
   * path's events along the way.
   *
   * @param path the path to follow, loaded with {@link PathGraph#fromPathFile(String)}
   * @return a path following command for the given path
   * @throws AutoBuilderException if the AutoBuilder has not been configured with {@link
   *     #configure}, or was configured for a drivetrain that is not holonomic
   */
  public static Command followPath(PathGraph path) {
    if (!isConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a path following command before being configured");
    }
    if (globals.pathGraphCommandBuilder == null) {
      throw new AutoBuilderException(
          "Following a 2027 path needs AutoBuilder.configure with a holonomic drivetrain. "
              + "configureCustom cannot follow 2027 paths.");
    }

    return globals.pathGraphCommandBuilder.apply(path);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return globals.pathfindToPoseCommandBuilder.build(pose, constraints, goalEndVelocity);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
    return pathfindToPose(pose, constraints, goalEndVelocity.in(MetersPerSecond));
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
    return pathfindToPose(pose, constraints, 0);
  }

  /**
   * Build a command to pathfind to a given pose that will be flipped based on the value of the path
   * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
   *     true
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPoseFlipped(
      Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
    Command pathfindToFlippedPose =
        pathfindToPose(FlippingUtil.flipFieldPose(pose), constraints, goalEndVelocity);
    Command pathfindToBluePose = pathfindToPose(pose, constraints, goalEndVelocity);
    BooleanSupplier shouldFlip = globals.shouldFlipPath;

    // Decide which side of the field to pathfind to when the command is run
    return Command.requiring(pathfindToBluePose.requirements())
        .executing(
            coroutine ->
                coroutine.await(
                    shouldFlip.getAsBoolean() ? pathfindToFlippedPose : pathfindToBluePose))
        .named("Pathfind to Pose (Auto Flipped)");
  }

  /**
   * Build a command to pathfind to a given pose that will be flipped based on the value of the path
   * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
   *     true
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPoseFlipped(
      Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
    return pathfindToPoseFlipped(pose, constraints, goalEndVelocity.in(MetersPerSecond));
  }

  /**
   * Build a command to pathfind to a given pose that will be flipped based on the value of the path
   * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
   *     true
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPoseFlipped(Pose2d pose, PathConstraints constraints) {
    return pathfindToPoseFlipped(pose, constraints, 0);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
   * drivetrain, the pose rotation delay distance will have no effect.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return globals.pathfindThenFollowPathCommandBuilder.apply(goalPath, pathfindingConstraints);
  }

  /**
   * Create and populate a selectable with every PathPlanner auto in the project. The default option
   * will be a command that does nothing.
   *
   * @return Selectable populated with all autos
   */
  public static Selectable<Command> buildAutoChooser() {
    return buildAutoChooser("");
  }

  /**
   * Create and populate a selectable with every PathPlanner auto in the project
   *
   * @param defaultAutoName The name of the auto that should be the default option. If this is an
   *     empty string, or if an auto with the given name does not exist, the default option will be
   *     a command that does nothing.
   * @return Selectable populated with all autos
   */
  public static Selectable<Command> buildAutoChooser(String defaultAutoName) {
    return buildAutoChooserWithOptionsModifier(defaultAutoName, (stream) -> stream);
  }

  /**
   * Create and populate a selectable with every PathPlanner auto in the project. The default option
   * will be a command that does nothing.
   *
   * @param optionsModifier A lambda function that can be used to modify the options before they go
   *     into the selectable
   * @return Selectable populated with all autos
   */
  public static Selectable<Command> buildAutoChooserWithOptionsModifier(
      Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
    return buildAutoChooserWithOptionsModifier("", optionsModifier);
  }

  /**
   * Create and populate a selectable with every PathPlanner auto in the project
   *
   * @param defaultAutoName The name of the auto that should be the default option. If this is an
   *     empty string, or if an auto with the given name does not exist, the default option will be
   *     a command that does nothing.
   * @param optionsModifier A lambda function that can be used to modify the options before they go
   *     into the selectable
   * @return Selectable populated with all autos
   */
  public static Selectable<Command> buildAutoChooserWithOptionsModifier(
      String defaultAutoName,
      Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    Selectable<Command> chooser = new Selectable<>();
    List<String> autoNames = getAllAutoNames();

    PathPlannerAuto defaultOption = null;
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);

      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

    if (defaultOption == null) {
      chooser.addDefault("None", CommandUtil.none());
    } else {
      chooser.addDefault(defaultOption.name(), defaultOption);
      chooser.add("None", CommandUtil.none());
    }

    optionsModifier.apply(options.stream()).forEach(auto -> chooser.add(auto.name(), auto));

    return chooser;
  }

  /**
   * Get a list of all auto names in the project
   *
   * @return List of all auto names
   */
  public static List<String> getAllAutoNames() {
    return AutoFile.getAllAutoNames();
  }

  /**
   * Get if AutoBuilder was configured for a holonomic drive train
   *
   * @return True if holonomic
   */
  public static boolean isHolonomic() {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException("AutoBuilder was not configured before use");
    }

    return globals.isHolonomic;
  }

  /**
   * Builds an auto command for the given auto name.
   *
   * @param autoName the name of the auto to build
   * @return an auto command for the given auto name
   */
  public static Command buildAuto(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  /**
   * Create a command to reset the robot's odometry to a given blue alliance pose
   *
   * @param bluePose The pose to reset to, relative to blue alliance origin
   * @return Command to reset the robot's odometry
   */
  public static Command resetOdom(Pose2d bluePose) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException("AutoBuilder was not configured before use");
    }

    BooleanSupplier shouldFlip = globals.shouldFlipPath;
    Consumer<Pose2d> resetPose = globals.resetPose;

    return Command.noRequirements(
            _ ->
                resetPose.accept(
                    shouldFlip.getAsBoolean() ? FlippingUtil.flipFieldPose(bluePose) : bluePose))
        .named("Reset Odometry");
  }

  @FunctionalInterface
  private interface PathfindToPoseBuilder {
    Command build(Pose2d pose, PathConstraints constraints, double goalEndVelocity);
  }

  /**
   * The static state of AutoBuilder, kept in one object so that {@link #resetForTesting()} resets
   * all of it.
   */
  private static class Globals {
    boolean configured = false;
    Supplier<Pose2d> poseSupplier;
    Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
    Function<PathGraph, Command> pathGraphCommandBuilder;
    Consumer<Pose2d> resetPose;
    BooleanSupplier shouldFlipPath;
    boolean isHolonomic;

    // Pathfinding builders
    boolean pathfindingConfigured = false;
    PathfindToPoseBuilder pathfindToPoseCommandBuilder;
    BiFunction<PathPlannerPath, PathConstraints, Command> pathfindThenFollowPathCommandBuilder;
  }
}
