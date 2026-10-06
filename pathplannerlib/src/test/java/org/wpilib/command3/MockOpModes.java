package org.wpilib.command3;

/**
 * Lets PathPlannerLib tests run the Commands v3 scheduler without the HAL, the same way WPILib's
 * own Commands v3 tests do. This has to be in the {@code org.wpilib.command3} package because the
 * opmode fetcher is package-private.
 */
public final class MockOpModes {
  private MockOpModes() {}

  /** Make the scheduler behave as if no opmode is running */
  public static void install() {
    OpModeFetcher.setFetcher(
        new OpModeFetcher() {
          @Override
          long getOpModeId() {
            return 0;
          }

          @Override
          String getOpModeName() {
            return "";
          }
        });
  }
}
