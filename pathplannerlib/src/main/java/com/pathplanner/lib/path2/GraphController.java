package com.pathplanner.lib.path2;

import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The controller that drives a robot through a list of waypoints. This is a port of the PathPlanner
 * 2027 app's {@code Path2PathFollower}, which the app uses to preview a path, so a robot following
 * a path asks for the same speeds the app simulated.
 *
 * <p>Every 20 ms loop the controller drives towards the target waypoint. A P controller on the
 * distance left along the path sets the speed, a second P controller pulls the robot back onto the
 * line between waypoints, and a heading controller turns the robot to the target heading. When the
 * robot is within the target waypoint's hand-off distance, the next waypoint becomes the target.
 *
 * <p>The returned speeds are unconstrained. Pass them through a {@link
 * com.pathplanner.lib.util.swerve.SwerveSetpointGenerator}, as the app does, before driving.
 */
public class GraphController {
  /** The loop period the controller is tuned for, in seconds */
  public static final double PERIOD_SECONDS = 0.02;

  private final PController translationController = new PController(4.0, PERIOD_SECONDS);
  private final PController crossTrackController = new PController(2.0, PERIOD_SECONDS);
  private final PController unprofiledRotationController = new PController(5.0, PERIOD_SECONDS);
  private final ProfiledPController rotationController;

  private List<Target> waypoints;
  private int targetWaypointIndex;
  private Translation2d segmentStart;
  private Translation2d segmentEnd;
  private Rotation2d heldHeading;
  private boolean usingUnprofiledRotation = false;

  /**
   * A waypoint the controller drives to
   *
   * @param waypoint The waypoint
   * @param handoffDistanceMeters The distance from this waypoint at which the next waypoint becomes
   *     the target, in meters. Zero for the last waypoint.
   */
  public record Target(GraphWaypoint waypoint, double handoffDistanceMeters) {
    Translation2d position() {
      return waypoint.position();
    }
  }

  /**
   * Create a controller
   *
   * @param waypoints The waypoints to drive through, starting with the waypoint the robot starts at
   * @param endToleranceMeters Distance tolerance at the last waypoint, in meters
   * @param endToleranceRadians Heading tolerance at the last waypoint, in radians
   * @param initialPose The current field pose of the robot
   * @param initialRobotRelativeSpeeds The current robot-relative speeds of the robot
   * @param targetFirstWaypoint Drive to the first waypoint before the second. The app's preview
   *     starts the robot on the first waypoint and passes false.
   */
  public GraphController(
      List<Target> waypoints,
      double endToleranceMeters,
      double endToleranceRadians,
      Pose2d initialPose,
      ChassisVelocities initialRobotRelativeSpeeds,
      boolean targetFirstWaypoint) {
    if (waypoints.isEmpty()) {
      throw new IllegalArgumentException("waypoints must not be empty");
    }
    this.waypoints = List.copyOf(waypoints);

    targetWaypointIndex = targetFirstWaypoint || waypoints.size() == 1 ? 0 : 1;
    segmentStart = targetFirstWaypoint ? initialPose.getTranslation() : waypoints.get(0).position();
    segmentEnd = waypoints.get(targetWaypointIndex).position();

    rotationController =
        new ProfiledPController(
            5.0, PERIOD_SECONDS, trapezoidConstraints(waypoints.get(targetWaypointIndex)));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(endToleranceRadians);
    rotationController.reset(
        initialPose.getRotation().getRadians(), initialRobotRelativeSpeeds.omega);

    translationController.setTolerance(endToleranceMeters);
    translationController.reset();
    crossTrackController.setTolerance(endToleranceMeters);
    crossTrackController.reset();
    unprofiledRotationController.enableContinuousInput(-Math.PI, Math.PI);
    unprofiledRotationController.setTolerance(endToleranceRadians);
    unprofiledRotationController.reset();
    updateHeldHeading(initialPose.getRotation());
  }

  /**
   * Get the index of the waypoint the robot is driving to
   *
   * @return The target waypoint index
   */
  public int getTargetWaypointIndex() {
    return targetWaypointIndex;
  }

  /**
   * Get the waypoints the controller is driving through
   *
   * @return The waypoints
   */
  public List<Target> getWaypoints() {
    return waypoints;
  }

  /**
   * Replace the waypoints after the target. This is used when a path branches, once the branch to
   * take is known. The waypoints up to and including the target must not change.
   *
   * @param newWaypoints The new waypoints
   */
  public void setWaypoints(List<Target> newWaypoints) {
    if (newWaypoints.size() <= targetWaypointIndex) {
      throw new IllegalArgumentException("Cannot remove the target waypoint");
    }
    waypoints = List.copyOf(newWaypoints);
    segmentEnd = waypoints.get(targetWaypointIndex).position();
  }

  /**
   * Check if the robot has reached the last waypoint, within the end tolerances
   *
   * @return True if the last waypoint has been reached
   */
  public boolean isFinished() {
    return targetWaypointIndex == waypoints.size() - 1
        && translationController.atSetpoint()
        && (usingUnprofiledRotation
            ? unprofiledRotationController.atSetpoint()
            : rotationController.atSetpoint());
  }

  /**
   * Calculate the robot-relative speeds for one 20 ms loop
   *
   * @param currentPose The current field pose of the robot
   * @param currentRobotRelativeSpeeds The current robot-relative speeds of the robot
   * @return The unconstrained robot-relative speeds to drive at
   */
  public ChassisVelocities calculate(
      Pose2d currentPose, ChassisVelocities currentRobotRelativeSpeeds) {
    advanceTargetIfNeeded(currentPose);
    Target activeWaypoint = waypoints.get(targetWaypointIndex);
    boolean useUnprofiledRotation =
        activeWaypoint.waypoint().isPointTowards() && activeWaypoint.waypoint().unprofiled();
    if (useUnprofiledRotation != usingUnprofiledRotation) {
      if (useUnprofiledRotation) {
        unprofiledRotationController.reset();
      } else {
        rotationController.reset(
            currentPose.getRotation().getRadians(), currentRobotRelativeSpeeds.omega);
      }
      usingUnprofiledRotation = useUnprofiledRotation;
    }

    rotationController.setConstraints(trapezoidConstraints(activeWaypoint));

    double remainingDistance = calculateRemainingPathDistance(currentPose);
    Translation2d toTarget = segmentEnd.minus(currentPose.getTranslation());
    double angleToTarget = Math.atan2(toTarget.getY(), toTarget.getX());

    double translationOutput = -translationController.calculate(remainingDistance, 0.0);
    double maxVelocity = activeWaypoint.waypoint().maxVelocityMPS();
    translationOutput = Math.clamp(translationOutput, -maxVelocity, maxVelocity);

    double vx = translationOutput * Math.cos(angleToTarget);
    double vy = translationOutput * Math.sin(angleToTarget);

    double crossTrackOutput =
        -crossTrackController.calculate(calculateCrossTrackError(currentPose), 0.0);
    double perpendicular = angleToTarget - Math.PI / 2.0;
    vx += crossTrackOutput * Math.cos(perpendicular);
    vy += crossTrackOutput * Math.sin(perpendicular);

    double targetHeading = rotationTarget(currentPose).getRadians();
    double rotationOutput =
        usingUnprofiledRotation
            ? unprofiledRotationController.calculate(
                currentPose.getRotation().getRadians(), targetHeading)
            : rotationController.calculate(currentPose.getRotation().getRadians(), targetHeading);

    return new ChassisVelocities(vx, vy, rotationOutput).toRobotRelative(currentPose.getRotation());
  }

  private void advanceTargetIfNeeded(Pose2d currentPose) {
    if (targetWaypointIndex >= waypoints.size() - 1) {
      return;
    }

    Target target = waypoints.get(targetWaypointIndex);
    double segmentLength = segmentStart.getDistance(target.position());
    double progress =
        calculateSegmentProgress(segmentStart, target.position(), currentPose.getTranslation());
    double handoffThreshold =
        segmentLength >= 1e-6
            ? Math.clamp(1.0 - target.handoffDistanceMeters() / segmentLength, 0.0, 1.0)
            : 1.0;
    boolean closeEnough =
        target.position().getDistance(currentPose.getTranslation())
            <= target.handoffDistanceMeters();

    if ((segmentLength >= 1e-6 && progress > handoffThreshold) || closeEnough) {
      targetWaypointIndex++;
      segmentStart = segmentEnd;
      segmentEnd = waypoints.get(targetWaypointIndex).position();
      updateHeldHeading(currentPose.getRotation());
    }
  }

  private Rotation2d rotationTarget(Pose2d currentPose) {
    GraphWaypoint activeWaypoint = waypoints.get(targetWaypointIndex).waypoint();
    Translation2d pointTowardsTarget = activeWaypoint.pointTowardsTarget();
    if (pointTowardsTarget != null) {
      Translation2d toTarget = pointTowardsTarget.minus(currentPose.getTranslation());
      if (toTarget.getNorm() <= 1e-9) {
        return currentPose.getRotation();
      }
      return Rotation2d.fromRadians(Math.atan2(toTarget.getY(), toTarget.getX()))
          .plus(activeWaypoint.pointTowardsRotationOffset());
    }

    for (int i = targetWaypointIndex; i < waypoints.size(); i++) {
      Rotation2d rotation = waypoints.get(i).waypoint().rotation();
      if (rotation != null) {
        return rotation;
      }
    }
    if (heldHeading == null) {
      heldHeading = currentPose.getRotation();
    }
    return heldHeading;
  }

  private void updateHeldHeading(Rotation2d currentHeading) {
    boolean hasFuturePose = false;
    for (int i = targetWaypointIndex; i < waypoints.size(); i++) {
      if (waypoints.get(i).waypoint().isPose()) {
        hasFuturePose = true;
        break;
      }
    }
    heldHeading = hasFuturePose ? null : currentHeading;
  }

  private static TrapezoidConstraints trapezoidConstraints(Target target) {
    return new TrapezoidConstraints(
        target.waypoint().maxAngularVelocityRadPerSec(),
        target.waypoint().maxAngularAccelerationRadPerSecSq());
  }

  private double calculateRemainingPathDistance(Pose2d currentPose) {
    Translation2d currentPosition = currentPose.getTranslation();
    double remainingDistance = 0.0;
    for (int i = targetWaypointIndex; i < waypoints.size(); i++) {
      remainingDistance += currentPosition.getDistance(waypoints.get(i).position());
      currentPosition = waypoints.get(i).position();
    }
    return remainingDistance;
  }

  private double calculateCrossTrackError(Pose2d currentPose) {
    double progress =
        calculateSegmentProgress(segmentStart, segmentEnd, currentPose.getTranslation());
    Translation2d closestPoint = segmentStart.interpolate(segmentEnd, progress);
    double pathVectorX = segmentEnd.getX() - segmentStart.getX();
    double pathVectorY = segmentEnd.getY() - segmentStart.getY();
    double robotVectorX = currentPose.getX() - segmentStart.getX();
    double robotVectorY = currentPose.getY() - segmentStart.getY();
    double crossProduct = pathVectorX * robotVectorY - pathVectorY * robotVectorX;

    double signedError = currentPose.getTranslation().getDistance(closestPoint);
    if (crossProduct < 0) {
      signedError = -signedError;
    }
    return signedError;
  }

  private static double calculateSegmentProgress(
      Translation2d segmentStart, Translation2d segmentEnd, Translation2d point) {
    double dx = segmentEnd.getX() - segmentStart.getX();
    double dy = segmentEnd.getY() - segmentStart.getY();
    double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < 1e-6) {
      return 0.0;
    }
    double pointDx = point.getX() - segmentStart.getX();
    double pointDy = point.getY() - segmentStart.getY();
    return Math.clamp((pointDx * dx + pointDy * dy) / lengthSquared, 0.0, 1.0);
  }

  private static double inputModulus(double input, double minimumInput, double maximumInput) {
    double modulus = maximumInput - minimumInput;
    // Truncating division, as the app's Dart code does
    long numMax = (long) ((input - minimumInput) / modulus);
    input -= numMax * modulus;
    long numMin = (long) ((input - maximumInput) / modulus);
    input -= numMin * modulus;
    return input;
  }

  /** A P controller with the same tolerance and continuous input behavior as the app's */
  private static class PController {
    private final double proportionalGain;
    private final double period;

    private double positionTolerance = 0.05;
    private double velocityTolerance = Double.POSITIVE_INFINITY;
    private double positionError = 0.0;
    private double velocityError = 0.0;
    private double previousError = 0.0;
    private boolean hasMeasurement = false;
    private boolean continuous = false;
    private double minimumInput = 0.0;
    private double maximumInput = 0.0;

    PController(double proportionalGain, double period) {
      this.proportionalGain = proportionalGain;
      this.period = period;
    }

    void setTolerance(double positionTolerance) {
      this.positionTolerance = positionTolerance;
      this.velocityTolerance = Double.POSITIVE_INFINITY;
    }

    void enableContinuousInput(double minimumInput, double maximumInput) {
      continuous = true;
      this.minimumInput = minimumInput;
      this.maximumInput = maximumInput;
    }

    void reset() {
      positionError = 0.0;
      velocityError = 0.0;
      previousError = 0.0;
      hasMeasurement = false;
    }

    double calculate(double measurement, double setpoint) {
      positionError = setpoint - measurement;
      if (continuous) {
        double errorBound = (maximumInput - minimumInput) / 2.0;
        positionError = inputModulus(positionError, -errorBound, errorBound);
      }
      velocityError = hasMeasurement ? (positionError - previousError) / period : 0.0;
      previousError = positionError;
      hasMeasurement = true;
      return proportionalGain * positionError;
    }

    boolean atSetpoint() {
      return hasMeasurement
          && Math.abs(positionError) < positionTolerance
          && Math.abs(velocityError) < velocityTolerance;
    }
  }

  /** A P controller that follows a trapezoid profile, matching the app's */
  private static class ProfiledPController {
    private final PController controller;
    private final double period;
    private TrapezoidConstraints constraints;
    private double setpointPosition = 0.0;
    private double setpointVelocity = 0.0;
    private boolean continuous = false;
    private double minimumInput = 0.0;
    private double maximumInput = 0.0;

    ProfiledPController(double proportionalGain, double period, TrapezoidConstraints constraints) {
      this.controller = new PController(proportionalGain, period);
      this.period = period;
      this.constraints = constraints;
    }

    void enableContinuousInput(double minimumInput, double maximumInput) {
      continuous = true;
      this.minimumInput = minimumInput;
      this.maximumInput = maximumInput;
    }

    void setConstraints(TrapezoidConstraints constraints) {
      this.constraints = constraints;
    }

    void setTolerance(double positionTolerance) {
      controller.setTolerance(positionTolerance);
    }

    void reset(double measurement, double velocity) {
      setpointPosition = measurement;
      setpointVelocity = velocity;
      controller.reset();
    }

    double calculate(double measurement, double goalPosition) {
      double goalVelocity = 0.0;
      if (continuous) {
        double errorBound = (maximumInput - minimumInput) / 2.0;
        goalPosition =
            inputModulus(goalPosition - measurement, -errorBound, errorBound) + measurement;
        setpointPosition =
            inputModulus(setpointPosition - measurement, -errorBound, errorBound) + measurement;
      }

      double[] next =
          trapezoidCalculate(
              constraints, period, setpointPosition, setpointVelocity, goalPosition, goalVelocity);
      setpointPosition = next[0];
      setpointVelocity = next[1];
      return controller.calculate(measurement, setpointPosition);
    }

    boolean atSetpoint() {
      return controller.atSetpoint();
    }
  }

  private record TrapezoidConstraints(double maxVelocity, double maxAcceleration) {}

  /**
   * The analytic trapezoid profile step used by the app and by WPILib's TrapezoidProfile.
   *
   * @return {position, velocity} after the given time
   */
  private static double[] trapezoidCalculate(
      TrapezoidConstraints constraints,
      double time,
      double currentPosition,
      double currentVelocity,
      double goalPosition,
      double goalVelocity) {
    double direction = currentPosition > goalPosition ? -1.0 : 1.0;
    currentPosition *= direction;
    currentVelocity *= direction;
    goalPosition *= direction;
    goalVelocity *= direction;

    currentVelocity =
        Math.clamp(currentVelocity, -constraints.maxVelocity(), constraints.maxVelocity());

    double cutoffBegin = currentVelocity / constraints.maxAcceleration();
    double cutoffDistanceBegin = cutoffBegin * cutoffBegin * constraints.maxAcceleration() / 2.0;
    double cutoffEnd = goalVelocity / constraints.maxAcceleration();
    double cutoffDistanceEnd = cutoffEnd * cutoffEnd * constraints.maxAcceleration() / 2.0;

    double fullTrapezoidDistance =
        cutoffDistanceBegin + (goalPosition - currentPosition) + cutoffDistanceEnd;
    double accelerationTime = constraints.maxVelocity() / constraints.maxAcceleration();
    double fullSpeedDistance =
        fullTrapezoidDistance - accelerationTime * accelerationTime * constraints.maxAcceleration();
    if (fullSpeedDistance < 0.0) {
      accelerationTime =
          Math.sqrt(Math.max(0.0, fullTrapezoidDistance) / constraints.maxAcceleration());
      fullSpeedDistance = 0.0;
    }

    double endAcceleration = accelerationTime - cutoffBegin;
    double endFullSpeed = endAcceleration + fullSpeedDistance / constraints.maxVelocity();
    double endDeceleration = endFullSpeed + accelerationTime - cutoffEnd;

    double position;
    double velocity;
    if (time < endAcceleration) {
      velocity = currentVelocity + time * constraints.maxAcceleration();
      position =
          currentPosition + (currentVelocity + time * constraints.maxAcceleration() / 2.0) * time;
    } else if (time < endFullSpeed) {
      position =
          currentPosition
              + (currentVelocity + constraints.maxVelocity()) / 2.0 * endAcceleration
              + constraints.maxVelocity() * (time - endAcceleration);
      velocity = constraints.maxVelocity();
    } else if (time <= endDeceleration) {
      double timeLeft = endDeceleration - time;
      velocity = goalVelocity + timeLeft * constraints.maxAcceleration();
      position = goalPosition - (goalVelocity + velocity) / 2.0 * timeLeft;
    } else {
      position = goalPosition;
      velocity = goalVelocity;
    }

    return new double[] {position * direction, velocity * direction};
  }
}
