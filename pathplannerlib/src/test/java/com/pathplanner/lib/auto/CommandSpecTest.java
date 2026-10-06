package com.pathplanner.lib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.CommandSpec.GroupType;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

class CommandSpecTest {
  private static JSONObject json(String text) throws Exception {
    return (JSONObject) new JSONParser().parse(text);
  }

  @Test
  void parsesAutoCommandTree() throws Exception {
    String text =
        """
        {
          "type": "sequential",
          "data": {
            "commands": [
              {"type": "path", "data": {"pathName": "First"}},
              {"type": "wait", "data": {"waitTime": 1.5}},
              {
                "type": "deadline",
                "data": {
                  "commands": [
                    {"type": "path", "data": {"pathName": "Second"}},
                    {"type": "named", "data": {"name": "Intake"}}
                  ]
                }
              },
              {
                "type": "parallel",
                "data": {"commands": [{"type": "race", "data": {"commands": []}}]}
              }
            ]
          }
        }
        """;

    CommandSpec spec = CommandSpec.fromJson(json(text), false);

    assertEquals(
        new CommandSpec.Group(
            GroupType.SEQUENTIAL,
            List.of(
                new CommandSpec.FollowPath("First", false),
                new CommandSpec.Wait(1.5),
                new CommandSpec.Group(
                    GroupType.DEADLINE,
                    List.of(
                        new CommandSpec.FollowPath("Second", false),
                        new CommandSpec.Named("Intake"))),
                new CommandSpec.Group(
                    GroupType.PARALLEL,
                    List.of(new CommandSpec.Group(GroupType.RACE, List.of()))))),
        spec);

    assertEquals(
        List.of(
            new CommandSpec.FollowPath("First", false),
            new CommandSpec.FollowPath("Second", false)),
        spec.getPathCommands());
  }

  @Test
  void parsesChoreoAutos() throws Exception {
    CommandSpec spec =
        CommandSpec.fromJson(
            json("{\"type\": \"path\", \"data\": {\"pathName\": \"Traj\"}}"), true);
    assertEquals(new CommandSpec.FollowPath("Traj", true), spec);
  }

  @Test
  void parsesChoreoWaitExpressions() throws Exception {
    CommandSpec spec =
        CommandSpec.fromJson(
            json(
                "{\"type\": \"wait\", \"data\": {\"waitTime\": {\"exp\": \"1 s\", \"val\": 1.0}}}"),
            true);
    assertEquals(new CommandSpec.Wait(1.0), spec);
  }

  @Test
  void unknownCommandTypesDoNothing() throws Exception {
    assertEquals(
        new CommandSpec.None(),
        CommandSpec.fromJson(json("{\"type\": \"other\", \"data\": {}}"), false));
  }

  @Test
  void parsesAutoFile() throws Exception {
    AutoFile auto =
        AutoFile.fromJson(
            json(
                """
                {
                  "version": "2025.0",
                  "command": {"type": "named", "data": {"name": "Score"}},
                  "resetOdom": true,
                  "folder": null,
                  "choreoAuto": false
                }
                """));

    assertEquals(new AutoFile(new CommandSpec.Named("Score"), true), auto);
  }
}
