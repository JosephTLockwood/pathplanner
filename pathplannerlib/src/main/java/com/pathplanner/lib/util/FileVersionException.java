package com.pathplanner.lib.util;

/** Exception for a mismatch between expected and actual file versions */
public class FileVersionException extends RuntimeException {
  private static final long serialVersionUID = 1L;

  /**
   * Create a new FileVersionException
   *
   * @param fileVersion The actual file version string
   * @param expectedVersion The expected file version string
   * @param fileName The name of the file with invalid version
   */
  public FileVersionException(String fileVersion, String expectedVersion, String fileName) {
    super(
        "Incompatible file version for '"
            + fileName
            + "'. Actual: '"
            + fileVersion
            + "' Expected: '"
            + expectedVersion
            + "'");
  }
}
