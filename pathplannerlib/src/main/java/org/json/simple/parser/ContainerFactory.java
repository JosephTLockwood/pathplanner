package org.json.simple.parser;

import java.util.List;
import java.util.Map;

public interface ContainerFactory {

  Map<String, Object> createObjectContainer();

  List<Object> creatArrayContainer();
}
