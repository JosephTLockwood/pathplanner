/**
 * PathPlannerLib support for the Commands v3 framework ({@code org.wpilib.command3}).
 *
 * <p>Robot projects using the Commands v3 vendordep should use the classes in this package in place
 * of the Commands v2 classes in {@link com.pathplanner.lib.auto} and {@link
 * com.pathplanner.lib.commands}. Paths, trajectories, controllers, and configuration classes are
 * shared between both command frameworks.
 *
 * <p>Commands in this package are written as coroutines. Path following commands {@link
 * org.wpilib.command3.Coroutine#yield() yield} once per loop, and the commands of event markers are
 * forked as child commands of the path following command, so they are canceled when the path ends.
 */
package com.pathplanner.lib.command3;
