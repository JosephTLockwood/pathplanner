package com.pathplanner.lib.command3;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.CommandSpec;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;

class CommandUtilTest extends CommandsV3TestBase {
  private final List<String> log = new ArrayList<>();

  private Command logged(String name, double seconds) {
    return Command.noRequirements(
            coroutine -> {
              log.add(name + " start");
              coroutine.wait(Seconds.of(seconds));
              log.add(name + " end");
            })
        .whenCanceled(() -> log.add(name + " canceled"))
        .named(name);
  }

  private Command build(CommandSpec spec) throws Exception {
    return CommandUtil.buildCommand(spec, false);
  }

  private void runToCompletion(Command command) {
    scheduler.schedule(command);
    step();
    stepUntil(() -> !isRunning(command), 500);
  }

  @Test
  void sequentialRunsCommandsInOrder() throws Exception {
    NamedCommands.registerCommand("A", logged("A", 0.1));
    NamedCommands.registerCommand("B", logged("B", 0.1));

    runToCompletion(
        build(new CommandSpec.Sequential(List.of(CommandSpec.named("A"), CommandSpec.named("B")))));

    assertEquals(List.of("A start", "A end", "B start", "B end"), log);
  }

  @Test
  void parallelWaitsForEveryCommand() throws Exception {
    NamedCommands.registerCommand("Short", logged("Short", 0.1));
    NamedCommands.registerCommand("Long", logged("Long", 0.5));

    runToCompletion(
        build(
            new CommandSpec.Parallel(
                List.of(CommandSpec.named("Short"), CommandSpec.named("Long")))));

    assertTrue(log.contains("Short end"));
    assertTrue(log.contains("Long end"));
    assertFalse(log.contains("Long canceled"));
  }

  @Test
  void raceEndsWhenAnyCommandEnds() throws Exception {
    NamedCommands.registerCommand("Short", logged("Short", 0.1));
    NamedCommands.registerCommand("Long", logged("Long", 0.5));

    runToCompletion(
        build(
            new CommandSpec.Race(List.of(CommandSpec.named("Long"), CommandSpec.named("Short")))));

    assertTrue(log.contains("Short end"));
    assertTrue(log.contains("Long canceled"));
  }

  @Test
  void deadlineEndsWhenFirstCommandEnds() throws Exception {
    NamedCommands.registerCommand("Deadline", logged("Deadline", 0.2));
    NamedCommands.registerCommand("Other", logged("Other", 1.0));

    runToCompletion(
        build(
            new CommandSpec.Deadline(
                List.of(CommandSpec.named("Deadline"), CommandSpec.named("Other")))));

    assertTrue(log.contains("Deadline end"));
    assertTrue(log.contains("Other canceled"));
  }

  @Test
  void deadlineWaitsForDeadlineEvenIfOthersFinishFirst() throws Exception {
    NamedCommands.registerCommand("Deadline", logged("Deadline", 0.5));
    NamedCommands.registerCommand("Other", logged("Other", 0.1));

    runToCompletion(
        build(
            new CommandSpec.Deadline(
                List.of(CommandSpec.named("Deadline"), CommandSpec.named("Other")))));

    assertTrue(log.indexOf("Other end") < log.indexOf("Deadline end"));
    assertFalse(log.contains("Deadline canceled"));
  }

  @Test
  void waitCommandWaitsForTime() throws Exception {
    Command wait = build(new CommandSpec.Wait(0.5));

    scheduler.schedule(wait);
    step();
    int loops = stepUntil(() -> !isRunning(wait), 100);
    assertEquals(0.5, (loops + 1) * LOOP_PERIOD, 1.5 * LOOP_PERIOD);
  }

  @Test
  void emptyGroupsFinishImmediately() throws Exception {
    for (CommandSpec spec :
        List.of(
            new CommandSpec.Sequential(List.of()),
            new CommandSpec.Parallel(List.of()),
            new CommandSpec.Race(List.of()),
            new CommandSpec.Deadline(List.of()),
            new CommandSpec.None())) {
      Command command = build(spec);
      scheduler.schedule(command);
      step();
      assertFalse(isRunning(command), spec + " should finish immediately");
    }
  }

  @Test
  void prebuiltCommandIsUsedDirectly() throws Exception {
    Command command = logged("Prebuilt", 0.1);
    assertSame(command, build(CommandSpec.of(command)));
  }

  @Test
  void prebuiltCommandFromOtherFrameworkIsRejected() {
    assertThrows(IllegalArgumentException.class, () -> build(CommandSpec.of("not a command")));
  }

  @Test
  void groupsRequireTheirCommandsMechanisms() throws Exception {
    var intake = new TestMechanism("Intake", scheduler);
    NamedCommands.registerCommand("Intake", runForever(intake, "Intake"));

    Command group =
        build(
            new CommandSpec.Sequential(
                List.of(new CommandSpec.Wait(0.1), CommandSpec.named("Intake"))));
    assertTrue(group.requirements().contains(intake));
  }
}
