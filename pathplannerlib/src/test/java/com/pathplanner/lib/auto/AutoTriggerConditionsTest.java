package com.pathplanner.lib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class AutoTriggerConditionsTest {
  private final AtomicReference<Pose2d> pose = new AtomicReference<>(Pose2d.ZERO);
  private final AtomicBoolean flip = new AtomicBoolean(false);

  private void setPosition(Translation2d position) {
    pose.set(new Pose2d(position, Rotation2d.ZERO));
  }

  @Test
  void inFieldAreaAutoFlippedUsesTheCurrentAllianceArea() {
    Translation2d blueMin = new Translation2d(1.0, 1.0);
    Translation2d blueMax = new Translation2d(3.0, 2.0);
    BooleanSupplier inArea =
        AutoTriggerConditions.inFieldAreaAutoFlipped(pose::get, flip::get, blueMin, blueMax);
    Translation2d blueCenter = new Translation2d(2.0, 1.5);
    Translation2d redCenter = FlippingUtil.flipFieldPosition(blueCenter);

    setPosition(blueCenter);
    assertTrue(inArea.getAsBoolean());
    setPosition(redCenter);
    assertFalse(inArea.getAsBoolean());

    flip.set(true);
    assertTrue(inArea.getAsBoolean());
    setPosition(blueCenter);
    assertFalse(inArea.getAsBoolean());
  }

  @Test
  void inFieldAreaRejectsInvalidBoundingBox() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            AutoTriggerConditions.inFieldArea(
                pose::get, new Translation2d(2.0, 2.0), new Translation2d(1.0, 3.0)));
  }

  @Test
  void nearFieldPositionAutoFlipped() {
    Translation2d bluePosition = new Translation2d(2.0, 3.0);
    BooleanSupplier near =
        AutoTriggerConditions.nearFieldPositionAutoFlipped(pose::get, flip::get, bluePosition, 0.5);

    setPosition(bluePosition);
    assertTrue(near.getAsBoolean());

    flip.set(true);
    assertFalse(near.getAsBoolean());
    setPosition(FlippingUtil.flipFieldPosition(bluePosition));
    assertTrue(near.getAsBoolean());
  }
}
