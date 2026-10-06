package com.pathplanner.lib;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.path.PathPlannerPath;
import java.io.IOException;
import java.lang.classfile.ClassFile;
import java.lang.classfile.constantpool.PoolEntry;
import java.lang.classfile.constantpool.Utf8Entry;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Robot projects can only use one of the Commands v2 or Commands v3 vendordeps, so the classes for
 * each command framework must not depend on the other one, and the classes shared by both must not
 * depend on either. This checks the references in the compiled classes of PathPlannerLib.
 */
class FrameworkIndependenceTest {
  private static final String LIB = "com/pathplanner/lib/";

  /** PathPlannerLib classes that use Commands v2 */
  private static final Set<String> V2_CLASSES =
      Set.of(
          LIB + "auto/AutoBuilder",
          LIB + "auto/CommandUtil",
          LIB + "auto/NamedCommands",
          LIB + "events/EventScheduler",
          LIB + "events/EventTrigger",
          LIB + "events/PointTowardsZoneTrigger");

  private static final String V2_PACKAGE = LIB + "commands/";
  private static final String V3_PACKAGE = LIB + "command3/";
  private static final String WPILIB_V2 = "org/wpilib/command2/";
  private static final String WPILIB_V3 = "org/wpilib/command3/";

  private static final Pattern CLASS_NAME =
      Pattern.compile("(org/wpilib/command[23]/|com/pathplanner/lib/)[A-Za-z0-9_/$]+");

  private enum Layer {
    CORE,
    V2,
    V3,
    OTHER
  }

  private static Layer layerOf(String className) {
    String topLevel =
        className.contains("$") ? className.substring(0, className.indexOf('$')) : className;
    if (topLevel.startsWith(V3_PACKAGE) || topLevel.startsWith(WPILIB_V3)) {
      return Layer.V3;
    } else if (topLevel.startsWith(V2_PACKAGE)
        || V2_CLASSES.contains(topLevel)
        || topLevel.startsWith(WPILIB_V2)) {
      return Layer.V2;
    } else if (topLevel.startsWith(LIB)) {
      return Layer.CORE;
    }
    return Layer.OTHER;
  }

  @Test
  void commandFrameworksAreIndependent() throws Exception {
    Map<String, Set<String>> references = readReferences();
    assertTrue(references.containsKey(LIB + "path/PathPlannerPath"), "Classes were not found");
    assertTrue(references.keySet().stream().anyMatch(c -> layerOf(c) == Layer.V3));

    List<String> violations = new ArrayList<>();
    references.forEach(
        (className, referenced) -> {
          Layer layer = layerOf(className);
          for (String ref : referenced) {
            Layer refLayer = layerOf(ref);
            boolean allowed =
                switch (layer) {
                  case CORE -> refLayer == Layer.CORE || refLayer == Layer.OTHER;
                  case V2 -> refLayer != Layer.V3;
                  case V3 -> refLayer != Layer.V2;
                  case OTHER -> true;
                };
            if (!allowed) {
              violations.add(
                  className + " (" + layer + ") references " + ref + " (" + refLayer + ")");
            }
          }
        });

    assertTrue(violations.isEmpty(), String.join("\n", violations));
  }

  /** Map of each PathPlannerLib class to the classes it references */
  private static Map<String, Set<String>> readReferences() throws IOException, URISyntaxException {
    Path classes =
        Path.of(PathPlannerPath.class.getProtectionDomain().getCodeSource().getLocation().toURI());
    Map<String, Set<String>> references = new HashMap<>();

    try (Stream<Path> files = Files.walk(classes.resolve(LIB))) {
      for (Path file : files.filter(f -> f.toString().endsWith(".class")).toList()) {
        String name = classes.relativize(file).toString().replace('\\', '/');
        Set<String> referenced = new TreeSet<>();
        for (PoolEntry entry : ClassFile.of().parse(file).constantPool()) {
          if (entry instanceof Utf8Entry utf8) {
            CLASS_NAME
                .matcher(utf8.stringValue())
                .results()
                .forEach(m -> referenced.add(m.group()));
          }
        }
        references.put(name.substring(0, name.length() - ".class".length()), referenced);
      }
    }

    return references;
  }
}
