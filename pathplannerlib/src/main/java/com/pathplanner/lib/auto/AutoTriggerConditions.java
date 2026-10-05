package com.pathplanner.lib.auto;

import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Conditions used by the triggers of PathPlannerAuto. These are independent of the command
 * framework, so they are shared between the Commands v2 and Commands v3 versions of
 * PathPlannerAuto.
 */
public final class AutoTriggerConditions {
  private AutoTriggerConditions() {}

  /**
   * Condition that is true when the given event will be reached within the given time
   *
   * @param eventName The name of the event
   * @param timeSeconds The time before the event, in seconds
   * @return Condition for the time before the event
   */
  public static BooleanSupplier beforeEvent(String eventName, double timeSeconds) {
    return () -> {
      var timeUntilEvent = ActivePathState.getTimeUntilEvent(eventName);
      return timeUntilEvent.isPresent() && timeUntilEvent.getAsDouble() < timeSeconds;
    };
  }

  /**
   * Condition that is true when the robot is within a distance of the start of the given event
   *
   * @param poseSupplier Supplier for the current robot pose
   * @param eventName The name of the event
   * @param distanceMeters The distance from the event, in meters
   * @return Condition for the distance from the event start
   */
  public static BooleanSupplier distanceFromEvent(
      Supplier<Pose2d> poseSupplier, String eventName, double distanceMeters) {
    return () ->
        isNearAny(
            poseSupplier.get(), ActivePathState.getEventStartPositions(eventName), distanceMeters);
  }

  /**
   * Condition that is true when the robot is within a distance of the end of the given event
   *
   * @param poseSupplier Supplier for the current robot pose
   * @param eventName The name of the event
   * @param distanceMeters The distance from the event end, in meters
   * @return Condition for the distance from the event end
   */
  public static BooleanSupplier distanceFromEventEnd(
      Supplier<Pose2d> poseSupplier, String eventName, double distanceMeters) {
    return () ->
        isNearAny(
            poseSupplier.get(), ActivePathState.getEventEndPositions(eventName), distanceMeters);
  }

  /**
   * Condition that is true when a path with the given name is being followed
   *
   * @param pathName The name of the path
   * @return Condition for the active path
   */
  public static BooleanSupplier activePath(String pathName) {
    return () -> pathName.equals(ActivePathState.getCurrentPathName());
  }

  /**
   * Condition that is true when the robot is near the given field position
   *
   * @param poseSupplier Supplier for the current robot pose
   * @param fieldPosition The field position
   * @param toleranceMeters The distance tolerance, in meters
   * @return Condition for the robot being near the position
   */
  public static BooleanSupplier nearFieldPosition(
      Supplier<Pose2d> poseSupplier, Translation2d fieldPosition, double toleranceMeters) {
    return () -> poseSupplier.get().getTranslation().getDistance(fieldPosition) <= toleranceMeters;
  }

  /**
   * Condition that is true when the robot is near the given field position, flipped to the red side
   * of the field when the path should be flipped
   *
   * @param poseSupplier Supplier for the current robot pose
   * @param shouldFlip Supplier that determines if positions should be flipped to the red side
   * @param blueFieldPosition The field position on the blue side of the field
   * @param toleranceMeters The distance tolerance, in meters
   * @return Condition for the robot being near the position
   */
  public static BooleanSupplier nearFieldPositionAutoFlipped(
      Supplier<Pose2d> poseSupplier,
      BooleanSupplier shouldFlip,
      Translation2d blueFieldPosition,
      double toleranceMeters) {
    Translation2d redFieldPosition = FlippingUtil.flipFieldPosition(blueFieldPosition);

    return () -> {
      Translation2d target = shouldFlip.getAsBoolean() ? redFieldPosition : blueFieldPosition;
      return poseSupplier.get().getTranslation().getDistance(target) <= toleranceMeters;
    };
  }

  /**
   * Condition that is true when the robot is within the given bounding box
   *
   * @param poseSupplier Supplier for the current robot pose
   * @param boundingBoxMin Minimum corner of the bounding box
   * @param boundingBoxMax Maximum corner of the bounding box
   * @return Condition for the robot being in the bounding box
   */
  public static BooleanSupplier inFieldArea(
      Supplier<Pose2d> poseSupplier, Translation2d boundingBoxMin, Translation2d boundingBoxMax) {
    requireValidBoundingBox(boundingBoxMin, boundingBoxMax);

    return () -> isInBox(poseSupplier.get(), boundingBoxMin, boundingBoxMax);
  }

  /**
   * Condition that is true when the robot is within the given bounding box, flipped to the red side
   * of the field when the path should be flipped
   *
   * @param poseSupplier Supplier for the current robot pose
   * @param shouldFlip Supplier that determines if positions should be flipped to the red side
   * @param blueBoundingBoxMin Minimum corner of the bounding box on the blue side of the field
   * @param blueBoundingBoxMax Maximum corner of the bounding box on the blue side of the field
   * @return Condition for the robot being in the bounding box
   */
  public static BooleanSupplier inFieldAreaAutoFlipped(
      Supplier<Pose2d> poseSupplier,
      BooleanSupplier shouldFlip,
      Translation2d blueBoundingBoxMin,
      Translation2d blueBoundingBoxMax) {
    requireValidBoundingBox(blueBoundingBoxMin, blueBoundingBoxMax);

    // Flipping can swap which corner is the minimum, so recompute the corners after flipping
    Translation2d flippedA = FlippingUtil.flipFieldPosition(blueBoundingBoxMin);
    Translation2d flippedB = FlippingUtil.flipFieldPosition(blueBoundingBoxMax);
    Translation2d redBoundingBoxMin =
        new Translation2d(
            Math.min(flippedA.getX(), flippedB.getX()), Math.min(flippedA.getY(), flippedB.getY()));
    Translation2d redBoundingBoxMax =
        new Translation2d(
            Math.max(flippedA.getX(), flippedB.getX()), Math.max(flippedA.getY(), flippedB.getY()));

    return () -> {
      if (shouldFlip.getAsBoolean()) {
        return isInBox(poseSupplier.get(), redBoundingBoxMin, redBoundingBoxMax);
      }
      return isInBox(poseSupplier.get(), blueBoundingBoxMin, blueBoundingBoxMax);
    };
  }

  private static boolean isNearAny(
      Pose2d pose, Iterable<Translation2d> positions, double distanceMeters) {
    for (Translation2d pos : positions) {
      if (pose.getTranslation().getDistance(pos) <= distanceMeters) {
        return true;
      }
    }
    return false;
  }

  private static boolean isInBox(Pose2d pose, Translation2d min, Translation2d max) {
    return pose.getX() >= min.getX()
        && pose.getY() >= min.getY()
        && pose.getX() <= max.getX()
        && pose.getY() <= max.getY();
  }

  private static void requireValidBoundingBox(Translation2d min, Translation2d max) {
    if (min.getX() >= max.getX() || min.getY() >= max.getY()) {
      throw new IllegalArgumentException(
          "Minimum bounding box position must have X and Y coordinates less than the maximum"
              + " bounding box position");
    }
  }
}
