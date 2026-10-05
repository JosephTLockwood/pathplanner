package com.pathplanner.lib.util;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventConditions;
import com.pathplanner.lib.follower.ActivePathState;
import java.util.ArrayList;
import java.util.List;

/** Utility class for testing code that uses Pathplanner lib */
public class PPLibTesting {
  private static final List<Runnable> resetHooks = new ArrayList<>();

  /**
   * Resets all static state to the values set at class initialization time.
   *
   * <p>This method should not be called during a competition. It makes a best-effort attempt to
   * reset the state, and may not update all static state.
   */
  public static void resetForTesting() {
    for (Runnable hook : List.copyOf(resetHooks)) {
      hook.run();
    }
    ActivePathState.setCurrentTrajectory(null);
    EventConditions.reset();
    PathPlannerLogging.clearLoggingCallbacks();
    PPHolonomicDriveController.clearFeedbackOverrides();
  }

  /**
   * Register a function that resets static state when {@link #resetForTesting()} is called. This is
   * used internally so that the static state of each command framework integration is reset without
   * this class depending on a command framework.
   *
   * @param hook Function that resets static state
   */
  public static void addResetHook(Runnable hook) {
    resetHooks.add(hook);
  }

  private PPLibTesting() {}
}
