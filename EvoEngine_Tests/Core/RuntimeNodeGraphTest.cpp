#include "NodeGraph.hpp"

#include <gtest/gtest.h>

using namespace evo_engine;

namespace {
using TestGraph = NodeGraph<int, int, int, int>;

void SerializeValue(YAML::Emitter& out, const int& value) {
  out << YAML::Key << "Value" << YAML::Value << value;
}

void DeserializeValue(const YAML::Node& in, int& value) {
  value = in["Value"].as<int>();
}
}  // namespace

TEST(RuntimeNodeGraph, PreservesGraphAndEditorLayoutThroughRuntimeSerialization) {
  const YAML::Node source = YAML::Load(R"(
Panning: [12.0, 34.0]
AutoPanningDelta: [-1.0, 2.0]
Nodes:
  - P: [100.0, 200.0]
    I: [0]
    O: [0]
    D: {Value: 3}
OutputPins:
  - L: [0]
    D: {Value: 4}
InputPins:
  - L: 0
    D: {Value: 5}
Links:
  - O: 0
    I: 0
    D: {Value: 6}
)");

  TestGraph graph;
  graph.Deserialize(source, DeserializeValue, DeserializeValue, DeserializeValue, DeserializeValue);

  ASSERT_EQ(graph.PeekNodes().size(), 1u);
  EXPECT_EQ(graph.PeekNode(0).data, 3);
  EXPECT_EQ(graph.PeekOutputPin(0).data, 4);
  EXPECT_EQ(graph.PeekInputPin(0).data, 5);
  EXPECT_EQ(graph.PeekLink(0).data, 6);

  YAML::Emitter out;
  out << YAML::BeginMap;
  graph.Serialize(out, SerializeValue, SerializeValue, SerializeValue, SerializeValue);
  out << YAML::EndMap;
  const YAML::Node roundtrip = YAML::Load(out.c_str());

  EXPECT_EQ(roundtrip["Panning"].as<glm::vec2>(), glm::vec2(12.0f, 34.0f));
  EXPECT_EQ(roundtrip["AutoPanningDelta"].as<glm::vec2>(), glm::vec2(-1.0f, 2.0f));
  EXPECT_EQ(roundtrip["Nodes"][0]["P"].as<glm::vec2>(), glm::vec2(100.0f, 200.0f));
  EXPECT_EQ(roundtrip["Nodes"][0]["D"]["Value"].as<int>(), 3);
  EXPECT_EQ(roundtrip["Links"][0]["D"]["Value"].as<int>(), 6);
}

TEST(RuntimeNodeGraph, ClearsStoredEditorPositionWhenReusingNodeHandle) {
  TestGraph graph;
  graph.Deserialize(
      YAML::Load("{Nodes: [{P: [100.0, 200.0], D: {Value: 3}}], OutputPins: [], InputPins: [], Links: []}"),
      DeserializeValue, DeserializeValue, DeserializeValue, DeserializeValue);
  graph.RecycleNode(0);
  EXPECT_EQ(graph.AllocateNode(0, 0), 0);

  YAML::Emitter out;
  out << YAML::BeginMap;
  graph.Serialize(out, SerializeValue, SerializeValue, SerializeValue, SerializeValue);
  out << YAML::EndMap;

  EXPECT_EQ(YAML::Load(out.c_str())["Nodes"][0]["P"].as<glm::vec2>(), glm::vec2{});
}

TEST(RuntimeNodeGraph, LayoutEditsRoundTripWithoutAnEditorContext) {
  TestGraph graph;
  const auto node = graph.AllocateNode(0, 0);
  graph.SetNodePosition(node, {321, 234});
  graph.SetPanning({17, -8});
  graph.SetAutoPanningDelta({2, 3});
  YAML::Emitter out;
  out << YAML::BeginMap;
  graph.Serialize(out, SerializeValue, SerializeValue, SerializeValue, SerializeValue);
  out << YAML::EndMap;
  TestGraph restored;
  restored.Deserialize(YAML::Load(out.c_str()), DeserializeValue, DeserializeValue, DeserializeValue, DeserializeValue);
  EXPECT_EQ(restored.GetNodePosition(node), glm::vec2(321, 234));
  EXPECT_EQ(restored.GetPanning(), glm::vec2(17, -8));
  EXPECT_EQ(restored.GetAutoPanningDelta(), glm::vec2(2, 3));
}
