package com.pathplanner.lib.command3;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.CommandSpec;
import com.pathplanner.lib.auto.CommandSpec.GroupType;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

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
        build(
            new CommandSpec.Group(
                GroupType.SEQUENTIAL,
                List.of(new CommandSpec.Named("A"), new CommandSpec.Named("B")))));

    assertEquals(List.of("A start", "A end", "B start", "B end"), log);
  }

  @Test
  void parallelWaitsForEveryCommand() throws Exception {
    NamedCommands.registerCommand("Short", logged("Short", 0.1));
    NamedCommands.registerCommand("Long", logged("Long", 0.5));

    runToCompletion(
        build(
            new CommandSpec.Group(
                GroupType.PARALLEL,
                List.of(new CommandSpec.Named("Short"), new CommandSpec.Named("Long")))));

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
            new CommandSpec.Group(
                GroupType.RACE,
                List.of(new CommandSpec.Named("Long"), new CommandSpec.Named("Short")))));

    assertTrue(log.contains("Short end"));
    assertTrue(log.contains("Long canceled"));
  }

  @Test
  void deadlineEndsWhenFirstCommandEnds() throws Exception {
    NamedCommands.registerCommand("Deadline", logged("Deadline", 0.2));
    NamedCommands.registerCommand("Other", logged("Other", 1.0));

    runToCompletion(
        build(
            new CommandSpec.Group(
                GroupType.DEADLINE,
                List.of(new CommandSpec.Named("Deadline"), new CommandSpec.Named("Other")))));

    assertTrue(log.contains("Deadline end"));
    assertTrue(log.contains("Other canceled"));
  }

  @Test
  void deadlineWaitsForDeadlineEvenIfOthersFinishFirst() throws Exception {
    NamedCommands.registerCommand("Deadline", logged("Deadline", 0.5));
    NamedCommands.registerCommand("Other", logged("Other", 0.1));

    runToCompletion(
        build(
            new CommandSpec.Group(
                GroupType.DEADLINE,
                List.of(new CommandSpec.Named("Deadline"), new CommandSpec.Named("Other")))));

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
            new CommandSpec.Group(GroupType.SEQUENTIAL, List.of()),
            new CommandSpec.Group(GroupType.PARALLEL, List.of()),
            new CommandSpec.Group(GroupType.RACE, List.of()),
            new CommandSpec.Group(GroupType.DEADLINE, List.of()),
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
    var intake = new Mechanism() {};
    NamedCommands.registerCommand("Intake", runForever(intake, "Intake"));

    Command group =
        build(
            new CommandSpec.Group(
                GroupType.SEQUENTIAL,
                List.of(new CommandSpec.Wait(0.1), new CommandSpec.Named("Intake"))));
    assertTrue(group.requirements().contains(intake));
  }
}
