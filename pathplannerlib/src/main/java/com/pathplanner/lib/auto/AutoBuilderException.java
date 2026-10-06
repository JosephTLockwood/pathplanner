package com.pathplanner.lib.auto;

/** An exception while building autos */
public class AutoBuilderException extends RuntimeException {
  private static final long serialVersionUID = 1L;

  /**
   * Create a new auto builder exception
   *
   * @param message Error message
   */
  public AutoBuilderException(String message) {
    super(message);
  }
}
