#include "MaizeTasselDescriptor.hpp"
#include "MaizeTassel.hpp"
#include <Application.hpp>
#include <EditorLayer.hpp>
#include <Scene.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>

using namespace l_system_plugin;
using namespace evo_engine;

namespace {
double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

MaizeTasselDescriptor::MaizeTasselDescriptor() {
  static const char* kDefaultMaizeTasselDescriptorYaml =
      R"MTASSEL_DEFAULTS(branch_node_count:
  mean: 12
  deviation: 4
branch_internode_length:
  mean:
    min_value: 0
    max_value: 3
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 1.5
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
branch_internode_thickness:
  mean:
    min_value: 0
    max_value: 0.35
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.884]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.615]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.033
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.238]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.704]
        - [0.1, 0]
lateral_insertion_angle:
  mean:
    min_value: 0
    max_value: 80
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.932]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.267]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 30
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.739]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.18]
        - [0.1, 0]
lateral_internode_length:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.836]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.5
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.841]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.18]
        - [0.1, 0]
lateral_node_count:
  mean:
    min_value: 8
    max_value: 24
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.811]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.257]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
peduncle_branch_probability:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.69]
        - [0.1, 0]
        - [-0.1, 0]
        - [0.2889306, 0.9586466]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.963]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
spike_node_count:
  mean: 36
  deviation: 6
spike_internode_length:
  mean:
    min_value: 0
    max_value: 0.5
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 1]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.857]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.15
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 1]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.277]
        - [0.1, 0]
spike_internode_thickness:
  mean:
    min_value: 0
    max_value: 0.35
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.703]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.462]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.035
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.722]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.383]
        - [0.1, 0]
spike_zone_branch_probability:
  mean:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_proximal_scale_x:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_proximal_scale_y:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_proximal_scale_z:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_proximal_angle:
  mean:
    min_value: 0
    max_value: 30
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.72]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.34]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 5
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_internode_length:
  mean:
    min_value: 0
    max_value: 0.75
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.685]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.391]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.5
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.713]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.311]
        - [0.1, 0]
main_pair_internode_thickness:
  mean:
    min_value: 0
    max_value: 0.25
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_internode_angle:
  mean:
    min_value: 0
    max_value: 60
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.788]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.307]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 30
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.785]
        - [0.1, 0]
        - [-0.10286678, 0.0016863407]
        - [1, 0.178]
        - [0.1, 0]
main_pair_distal_scale_x:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_distal_scale_y:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_distal_scale_z:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
main_pair_distal_angle:
  mean:
    min_value: 0
    max_value: 40
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
branch_pair_proximal_scale_x:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
branch_pair_proximal_scale_y:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
 )MTASSEL_DEFAULTS"
      R"MTASSEL_DEFAULTS(       - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
branch_pair_proximal_scale_z:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
branch_pair_proximal_angle:
  mean:
    min_value: 0
    max_value: 45
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.641]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.319]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 15
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.728]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.132]
        - [0.1, 0]
branch_pair_internode_length:
  mean:
    min_value: 0
    max_value: 1.5
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.766]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.265]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.05
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.704]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.256]
        - [0.1, 0]
branch_pair_internode_thickness:
  mean:
    min_value: 0
    max_value: 0.4
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.7]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.47]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
branch_pair_internode_angle:
  mean:
    min_value: 0
    max_value: 45
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.644]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.241]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 15
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.738]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.386]
        - [0.1, 0]
branch_pair_distal_scale_x:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.509]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.509]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.501]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.51]
        - [0.1, 0]
branch_pair_distal_scale_y:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.507]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.501]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.501]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.505]
        - [0.1, 0]
branch_pair_distal_scale_z:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.495]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.507]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0.1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.505]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.491]
        - [0.1, 0]
branch_pair_distal_angle:
  mean:
    min_value: 0
    max_value: 32
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.656]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.195]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 15
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
lateral_initiation_delay_gdd:
  mean:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
spike_anthesis_offset_gdd:
  mean:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
primary_lateral_branch_probability:
  mean:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
secondary_lateral_branch_probability:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.64]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.263]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
phyllotaxis_angle:
  mean: 180
  deviation: 0
branch_azimuth_offset:
  mean: 15
  deviation: 15
lateral_thickness_ratio:
  mean: 0.5
  deviation: 0.08
final_age_gdd:
  mean: 1500
  deviation: 150
secondary_insertion_angle:
  mean: 60
  deviation: 0
secondary_internode_length:
  mean: 2
  deviation: 0
secondary_internode_thickness:
  mean: 0.1
  deviation: 0
secondary_node_count:
  mean: 6
  deviation: 18
rachis_elongation_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
rachis_thickness_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
lateral_elongation_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
lateral_thickness_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
lateral_angle_development_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_proximal_scale_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_proximal_angle_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_internode_length_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_internode_thickness_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_internode_angle_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_distal_scale_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
pair_distal_angle_curve:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 0]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 1]
    - [0.1, 0]
base_temperature: 10
plastochron_gdd: 100
anthesis_gdd: 500
max_gdd: 1500
live_preview: false
live_preview_rate_hz: 12
grid_rows: 3
grid_cols: 6
grid_spacing: 45
tropism_count: 3
tropism_0_dir_x:
  mean: 0
  deviation: 0
tropism_0_dir_y:
  mean: -1
  deviation: 0
tropism_0_dir_z:
  mean: 0
  deviation: 0
tropism_0_strength:
  mean: 8
  deviation: 2
tropism_0_usage_chance_percent: 50
tropism_0_order_response:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.293]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.648]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
tropism_1_dir_x:
  mean: 0
  deviation: 0
tropism_1_dir_y:
  mean: -1
  deviation: 0
tropism_1_dir_z:
  mean: 1
  deviation: 0
tropism_1_strength:
  mean: 0
  deviation: 5
tropism_1_usage_chance_percent: 25
tropism_1_order_response:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
tropism_2_dir_x:
  mean: 0
  deviation: 0
tropism_2_dir_y:
  mean: -1
  deviati)MTASSEL_DEFAULTS"
      R"MTASSEL_DEFAULTS(on: 0
tropism_2_dir_z:
  mean: -1
  deviation: 0
tropism_2_strength:
  mean: 20
  deviation: 0
tropism_2_usage_chance_percent: 5
tropism_2_order_response:
  mean:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 1
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.5]
        - [0.1, 0])MTASSEL_DEFAULTS";
  try {
    const YAML::Node defaults = YAML::Load(kDefaultMaizeTasselDescriptorYaml);
    if (defaults && defaults.IsMap()) {
      Deserialize(defaults);
    }
  } catch (const YAML::Exception&) {
    // Keep inline member defaults if the embedded YAML payload fails to parse.
  }
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

SampledTasselParams MaizeTasselDescriptor::Sample(std::mt19937& rng) const {
  SampledTasselParams p;

  // Branch zone (lower rachis).
  p.branch_node_count = std::max(0, static_cast<int>(std::round(SampleDistribution(branch_node_count, rng))));
  p.branch_internode_length = branch_internode_length;
  p.branch_internode_thickness = branch_internode_thickness;
  p.lateral_insertion_angle = lateral_insertion_angle;
  p.lateral_internode_length = lateral_internode_length;
  p.lateral_node_count = lateral_node_count;
  p.peduncle_branch_probability = peduncle_branch_probability;

  // Central spike (upper rachis).
  p.spike_node_count = std::max(0, static_cast<int>(std::round(SampleDistribution(spike_node_count, rng))));
  p.spike_internode_length = spike_internode_length;
  p.spike_internode_thickness = spike_internode_thickness;
  p.spike_zone_branch_probability = spike_zone_branch_probability;

  // Main-rachis pair morphology.
  p.main_pair_proximal_scale_x = main_pair_proximal_scale_x;
  p.main_pair_proximal_scale_y = main_pair_proximal_scale_y;
  p.main_pair_proximal_scale_z = main_pair_proximal_scale_z;
  p.main_pair_proximal_angle = main_pair_proximal_angle;
  p.main_pair_internode_length = main_pair_internode_length;
  p.main_pair_internode_thickness = main_pair_internode_thickness;
  p.main_pair_internode_angle = main_pair_internode_angle;
  p.main_pair_distal_scale_x = main_pair_distal_scale_x;
  p.main_pair_distal_scale_y = main_pair_distal_scale_y;
  p.main_pair_distal_scale_z = main_pair_distal_scale_z;
  p.main_pair_distal_angle = main_pair_distal_angle;

  // Non-main-axis pair morphology.
  p.branch_pair_proximal_scale_x = branch_pair_proximal_scale_x;
  p.branch_pair_proximal_scale_y = branch_pair_proximal_scale_y;
  p.branch_pair_proximal_scale_z = branch_pair_proximal_scale_z;
  p.branch_pair_proximal_angle = branch_pair_proximal_angle;
  p.branch_pair_internode_length = branch_pair_internode_length;
  p.branch_pair_internode_thickness = branch_pair_internode_thickness;
  p.branch_pair_internode_angle = branch_pair_internode_angle;
  p.branch_pair_distal_scale_x = branch_pair_distal_scale_x;
  p.branch_pair_distal_scale_y = branch_pair_distal_scale_y;
  p.branch_pair_distal_scale_z = branch_pair_distal_scale_z;
  p.branch_pair_distal_angle = branch_pair_distal_angle;

  // Branch timing and branch probabilities.
  p.lateral_initiation_delay_gdd = lateral_initiation_delay_gdd;
  p.spike_anthesis_offset_gdd = spike_anthesis_offset_gdd;
  p.primary_lateral_branch_probability = primary_lateral_branch_probability;
  p.secondary_lateral_branch_probability = secondary_lateral_branch_probability;

  // Shared.
  p.phyllotaxis_angle = SampleDistribution(phyllotaxis_angle, rng);
  p.branch_azimuth_offset = branch_azimuth_offset;
  p.lateral_thickness_ratio = std::max(0.01f, SampleDistribution(lateral_thickness_ratio, rng));
  p.final_age_gdd = final_age_gdd;
  p.final_age_gdd.mean = std::max(1.0f, p.final_age_gdd.mean);
  p.final_age_gdd.deviation = std::max(0.0f, p.final_age_gdd.deviation);

  // Secondary branches.
  p.secondary_insertion_angle = std::max(1.0f, SampleDistribution(secondary_insertion_angle, rng));
  p.secondary_internode_length = std::max(0.01f, SampleDistribution(secondary_internode_length, rng));
  p.secondary_internode_thickness = std::max(0.01f, SampleDistribution(secondary_internode_thickness, rng));
  p.secondary_node_count = std::max(0, static_cast<int>(std::round(SampleDistribution(secondary_node_count, rng))));

  // Growth curves.
  p.rachis_elongation_curve = rachis_elongation_curve;
  p.rachis_thickness_curve = rachis_thickness_curve;
  p.lateral_elongation_curve = lateral_elongation_curve;
  p.lateral_thickness_curve = lateral_thickness_curve;
  p.lateral_angle_development_curve = lateral_angle_development_curve;
  p.pair_proximal_scale_curve = pair_proximal_scale_curve;
  p.pair_proximal_angle_curve = pair_proximal_angle_curve;
  p.pair_internode_length_curve = pair_internode_length_curve;
  p.pair_internode_thickness_curve = pair_internode_thickness_curve;
  p.pair_internode_angle_curve = pair_internode_angle_curve;
  p.pair_distal_scale_curve = pair_distal_scale_curve;
  p.pair_distal_angle_curve = pair_distal_angle_curve;

  // Tropisms.
  for (const auto& entry : tropisms) {
    const float usage_chance = std::clamp(entry.usage_chance_percent, 0.0f, 100.0f);
    if (SampleUnit01(rng) * 100.0f > usage_chance) {
      continue;
    }

    SampledTropism st;
    const float dx = SampleDistribution(entry.direction_x, rng);
    const float dy = SampleDistribution(entry.direction_y, rng);
    const float dz = SampleDistribution(entry.direction_z, rng);
    const glm::vec3 dir(dx, dy, dz);
    const float len = glm::length(dir);
    st.direction = (len > 0.001f) ? dir / len : glm::vec3(0.0f, -1.0f, 0.0f);
    st.strength = SampleDistribution(entry.strength, rng);
    st.order_response = entry.order_response;
    p.tropisms.push_back(std::move(st));
  }

  // Thermal timing.
  p.plastochron_gdd = plastochron_gdd;
  p.anthesis_gdd = anthesis_gdd;
  p.maturity_gdd = maturity_gdd;

  return p;
}

// ---------------------------------------------------------------------------
// Instantiate — create entity with MaizeTassel component
// ---------------------------------------------------------------------------

Entity MaizeTasselDescriptor::Instantiate() const {
  const auto scene = Application::GetActiveScene();
  if (!scene)
    return {};

  const auto entity = scene->CreateEntity(GetTitle());
  const auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
  tassel->descriptor_ref = GetSelf();
  tassel->GenerateGeometryEntities();

  return entity;
}

// ---------------------------------------------------------------------------
// Inspector UI
// ---------------------------------------------------------------------------

bool MaizeTasselDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  bool editor_preferences_changed = false;

  // -- Instantiation controls --
  if (ImGui::Button("Instantiate")) {
    editor_layer->SetSelectedEntity(Instantiate());
  }

  ImGui::SameLine();
  if (ImGui::Checkbox("Live Preview", &live_preview)) {
    editor_preferences_changed = true;
    if (!live_preview) {
      live_preview_dirty_ = false;
      live_preview_was_dragging_ = false;
      live_preview_needs_full_apply_ = false;
    }
  }

  if (ImGui::DragFloat("Live Preview Rate (Hz)",
                       &live_preview_rate_hz,
                       0.25f,
                       1.0f,
                       60.0f,
                       "%.1f")) {
    live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
    editor_preferences_changed = true;
  }

  if (ImGui::Checkbox("Representative Only While Dragging", &live_preview_representative_only)) {
    editor_preferences_changed = true;
  }

  if (ImGui::Checkbox("Cap Preview Target GDD", &live_preview_cap_target_gdd)) {
    editor_preferences_changed = true;
  }

  if (ImGui::DragFloat("Preview Max GDD",
                       &live_preview_max_gdd,
                       5.0f,
                       0.0f,
                       5000.0f,
                       "%.1f")) {
    live_preview_max_gdd = std::max(0.0f, live_preview_max_gdd);
    editor_preferences_changed = true;
  }

  if (ImGui::DragInt("Preview Max Growth Steps",
                     &live_preview_max_growth_steps,
                     1.0f,
                     1,
                     10000)) {
    live_preview_max_growth_steps = std::clamp(live_preview_max_growth_steps, 1, 10000);
    editor_preferences_changed = true;
  }

  if (live_preview_apply_count_ > 0) {
    const double avg_apply_ms = live_preview_total_apply_ms_ /
                                static_cast<double>(live_preview_apply_count_);
    ImGui::Text("Preview last/avg ms: %.3f / %.3f",
                live_preview_last_apply_ms_,
                avg_apply_ms);
  }
  ImGui::Text("Preview requests/applied/coalesced: %u / %u / %u",
              live_preview_request_count_,
              live_preview_apply_count_,
              live_preview_coalesced_count_);

  if (ImGui::SmallButton("Reset Preview Stats")) {
    live_preview_request_count_ = 0;
    live_preview_apply_count_ = 0;
    live_preview_coalesced_count_ = 0;
    live_preview_last_apply_ms_ = 0.0;
    live_preview_total_apply_ms_ = 0.0;
  }

  // -- Grid instantiation --
  if (ImGui::TreeNodeEx("Grid Instantiate")) {
    ImGui::DragInt("Rows", &grid_rows, 1, 1, 50);
    ImGui::DragInt("Cols", &grid_cols, 1, 1, 50);
    ImGui::DragFloat("Spacing", &grid_spacing, 0.1f, 0.5f, 50.0f);

    if (ImGui::Button("Instantiate Grid")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Tassel Grid");
        const float offset_y = (static_cast<float>(grid_rows) - 1.0f) * grid_spacing * 0.5f;
        const float offset_z = (static_cast<float>(grid_cols) - 1.0f) * grid_spacing * 0.5f;
        const auto base_seed = static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < grid_rows; i++) {
          for (int j = 0; j < grid_cols; j++) {
            const auto entity = scene->CreateEntity(GetTitle() + " [" + std::to_string(i) + "," + std::to_string(j) + "]");
            const auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
            tassel->descriptor_ref = GetSelf();
            tassel->seed = base_seed + static_cast<unsigned int>(i * grid_cols + j);

            scene->SetParent(entity, container, false);

            Transform transform;
            transform.SetPosition(glm::vec3(
                0.0f,
                static_cast<float>(i) * grid_spacing - offset_y,
                static_cast<float>(j) * grid_spacing - offset_z));
            scene->SetDataComponent(entity, transform);

            tassel->GenerateGeometryEntities();
          }
        }
      }
    }

    ImGui::SameLine();
    if (ImGui::Button("Delete Grid")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
        if (tassel_entities_ptr) {
          // Copy before iterating to avoid dangling pointer if p_owners_collections_list_ reallocates.
          const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
          // Collect matching tassel entities and their container parents.
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : tassel_entities) {
            if (!scene->IsEntityValid(entity))
              continue;
            auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
            if (!tassel)
              continue;
            if (tassel->descriptor_ref.Get<MaizeTasselDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Tassel Grid") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete) {
            scene->DeleteEntity(entity);
          }
          // Deduplicate and delete now-empty container entities.
          std::sort(containers.begin(), containers.end(),
                    [](const Entity& a, const Entity& b) { return a.GetIndex() < b.GetIndex(); });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container))
              scene->DeleteEntity(container);
          }
        }
      }
    }

    ImGui::TreePop();
  }

  ImGui::Separator();

  // -- Parameter Space Explorer --
  if (ImGui::TreeNodeEx("Parameter Space Explorer")) {
    if (!explorer_.IsBound()) explorer_.Bind(*this);
    if (explorer_.OnInspect()) {
      changed = true;
    }
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Main Axis (Peduncle + Spike Zone)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= branch_node_count.OnInspect("Peduncle Node Count", 0.5f);
    changed |= branch_internode_length.OnInspect("Peduncle Internode Length (Position)");
    changed |= branch_internode_thickness.OnInspect("Peduncle Internode Thickness (Position)");

    changed |= spike_node_count.OnInspect("Spike-Zone Node Count", 0.5f);
    changed |= spike_internode_length.OnInspect("Spike-Zone Internode Length (Position)");
    changed |= spike_internode_thickness.OnInspect("Spike-Zone Internode Thickness (Position)");

    ImGui::PushID("curve_rachis_elongation");
    ImGui::TextUnformatted("Age Curve: Main-axis internode elongation progression");
    changed |= rachis_elongation_curve.OnInspect("Main Axis Length Growth");
    ImGui::PopID();

    ImGui::PushID("curve_rachis_thickness");
    ImGui::TextUnformatted("Age Curve: Main-axis internode thickness progression");
    changed |= rachis_thickness_curve.OnInspect("Main Axis Thickness Growth");
    ImGui::PopID();

    changed |= phyllotaxis_angle.OnInspect("Base Phyllotaxis Angle", 1.0f);
    changed |= branch_azimuth_offset.OnInspect("Branch Azimuth Offset", 0.5f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Main Stem Branching (Peduncle)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= peduncle_branch_probability.OnInspect("Peduncle Branch Probability (Position)");
    changed |= lateral_initiation_delay_gdd.OnInspect("Lateral Initiation Delay (GDD)");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Main Stem Branching (Spike Zone)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= spike_zone_branch_probability.OnInspect("Spike-Zone Branch Probability (Position)");
    changed |= spike_anthesis_offset_gdd.OnInspect("Anthesis Offset (GDD)");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Primary Lateral Branches", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= lateral_insertion_angle.OnInspect("Insertion Angle (Position)");
    changed |= lateral_internode_length.OnInspect("Internode Length (Position)");
    changed |= lateral_node_count.OnInspect("Relative Node Count (Position)");
    changed |= primary_lateral_branch_probability.OnInspect("Primary->Secondary Branch Probability (Position)");
    changed |= lateral_thickness_ratio.OnInspect("Lateral Thickness Ratio", 0.01f);

    ImGui::PushID("curve_lateral_elongation");
    ImGui::TextUnformatted("Age Curve: Lateral internode elongation progression");
    changed |= lateral_elongation_curve.OnInspect("Lateral Length Growth");
    ImGui::PopID();

    ImGui::PushID("curve_lateral_thickness");
    ImGui::TextUnformatted("Age Curve: Lateral internode thickness progression");
    changed |= lateral_thickness_curve.OnInspect("Lateral Thickness Growth");
    ImGui::PopID();

    ImGui::PushID("curve_lateral_angle");
    ImGui::TextUnformatted("Age Curve: Lateral insertion angle opening progression");
    changed |= lateral_angle_development_curve.OnInspect("Lateral Angle Growth");
    ImGui::PopID();

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Secondary Branches", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= secondary_lateral_branch_probability.OnInspect("Secondary->Secondary Branch Probability (Position)");
    changed |= secondary_insertion_angle.OnInspect("Secondary Insertion Angle", 0.5f);
    changed |= secondary_internode_length.OnInspect("Secondary Internode Length", 0.1f);
    changed |= secondary_internode_thickness.OnInspect("Secondary Internode Thickness", 0.01f);
    changed |= secondary_node_count.OnInspect("Secondary Relative Node Count", 0.5f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Spikelet Pair Morphology (Main Rachis)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= main_pair_proximal_scale_x.OnInspect("Proximal Ellipsoid Scale X (Position)");
    changed |= main_pair_proximal_scale_y.OnInspect("Proximal Ellipsoid Scale Y (Position)");
    changed |= main_pair_proximal_scale_z.OnInspect("Proximal Ellipsoid Scale Z (Position)");
    changed |= main_pair_proximal_angle.OnInspect("Proximal Ellipsoid Branch Angle (Position)");
    changed |= main_pair_internode_length.OnInspect("Pair Internode Length (Position)");
    changed |= main_pair_internode_thickness.OnInspect("Pair Internode Thickness (Position)");
    changed |= main_pair_internode_angle.OnInspect("Pair Internode Branch Angle (Position)");
    changed |= main_pair_distal_scale_x.OnInspect("Distal Ellipsoid Scale X (Position)");
    changed |= main_pair_distal_scale_y.OnInspect("Distal Ellipsoid Scale Y (Position)");
    changed |= main_pair_distal_scale_z.OnInspect("Distal Ellipsoid Scale Z (Position)");
    changed |= main_pair_distal_angle.OnInspect("Distal Ellipsoid Branch Angle (Position)");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Spikelet Pair Morphology (Peduncle Branches)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= branch_pair_proximal_scale_x.OnInspect("Proximal Ellipsoid Scale X (Position)");
    changed |= branch_pair_proximal_scale_y.OnInspect("Proximal Ellipsoid Scale Y (Position)");
    changed |= branch_pair_proximal_scale_z.OnInspect("Proximal Ellipsoid Scale Z (Position)");
    changed |= branch_pair_proximal_angle.OnInspect("Proximal Ellipsoid Branch Angle (Position)");
    changed |= branch_pair_internode_length.OnInspect("Pair Internode Length (Position)");
    changed |= branch_pair_internode_thickness.OnInspect("Pair Internode Thickness (Position)");
    changed |= branch_pair_internode_angle.OnInspect("Pair Internode Branch Angle (Position)");
    changed |= branch_pair_distal_scale_x.OnInspect("Distal Ellipsoid Scale X (Position)");
    changed |= branch_pair_distal_scale_y.OnInspect("Distal Ellipsoid Scale Y (Position)");
    changed |= branch_pair_distal_scale_z.OnInspect("Distal Ellipsoid Scale Z (Position)");
    changed |= branch_pair_distal_angle.OnInspect("Distal Ellipsoid Branch Angle (Position)");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Spikelet Pair Growth Curves", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= pair_proximal_scale_curve.OnInspect("Proximal Scale Growth");
    changed |= pair_proximal_angle_curve.OnInspect("Proximal Angle Growth");
    changed |= pair_internode_length_curve.OnInspect("Pair Internode Length Growth");
    changed |= pair_internode_thickness_curve.OnInspect("Pair Internode Thickness Growth");
    changed |= pair_internode_angle_curve.OnInspect("Pair Internode Angle Growth");
    changed |= pair_distal_scale_curve.OnInspect("Distal Scale Growth");
    changed |= pair_distal_angle_curve.OnInspect("Distal Angle Growth");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Tropisms")) {
    if (ImGui::Button("+ Add Tropism")) {
      tropisms.emplace_back();
      changed = true;
    }
    int remove_idx = -1;
    for (int i = 0; i < static_cast<int>(tropisms.size()); i++) {
      ImGui::PushID(i);
      const std::string header = "Tropism " + std::to_string(i);
      if (ImGui::TreeNodeEx(header.c_str())) {
        changed |= tropisms[i].direction_x.OnInspect("Direction X", 0.01f);
        changed |= tropisms[i].direction_y.OnInspect("Direction Y", 0.01f);
        changed |= tropisms[i].direction_z.OnInspect("Direction Z", 0.01f);
        changed |= tropisms[i].strength.OnInspect("Strength", 0.01f);
        if (ImGui::DragFloat("Usage Chance (%)",
                             &tropisms[i].usage_chance_percent,
                             0.5f,
                             0.0f,
                             100.0f,
                             "%.1f")) {
          tropisms[i].usage_chance_percent = std::clamp(tropisms[i].usage_chance_percent, 0.0f, 100.0f);
          changed = true;
        }
        changed |= tropisms[i].order_response.OnInspect("Order Response");
        if (ImGui::Button("Remove")) {
          remove_idx = i;
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    if (remove_idx >= 0) {
      tropisms.erase(tropisms.begin() + remove_idx);
      changed = true;
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Global Development", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= ImGui::DragFloat("Base Temperature", &base_temperature, 0.5f, 0.0f, 30.0f);
    changed |= ImGui::DragFloat("Plastochron GDD", &plastochron_gdd, 1.0f, 1.0f, 200.0f);
    changed |= ImGui::DragFloat("Anthesis GDD", &anthesis_gdd, 5.0f, 10.0f, 1000.0f);
    changed |= ImGui::DragFloat("Internode Maturity GDD", &maturity_gdd, 5.0f, 10.0f, 2000.0f);
    changed |= final_age_gdd.OnInspect("Spikelet Pair Final Age GDD", 1.0f);
    ImGui::TreePop();
  }

  // -- Live preview: coalesce edits and regenerate matching tassels at capped cadence --
  if (changed && live_preview) {
    live_preview_dirty_ = true;
    live_preview_request_count_++;
  }

  const bool drag_active = ImGui::IsMouseDown(ImGuiMouseButton_Left) && ImGui::IsAnyItemActive();
  const bool drag_ended = live_preview_was_dragging_ && !drag_active;
  live_preview_was_dragging_ = live_preview && drag_active;

  if (drag_ended && live_preview && live_preview_needs_full_apply_) {
    // Commit drag-session preview edits to every matching tassel when the drag ends.
    live_preview_dirty_ = true;
  }

  if (changed || editor_preferences_changed) {
    SetUnsaved();
  }

  if (live_preview && live_preview_dirty_) {
    live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
    const double now_seconds = GetSteadyTimeSeconds();
    const double min_interval_seconds = 1.0 / static_cast<double>(live_preview_rate_hz);

    const bool throttle_ready = live_preview_last_apply_seconds_ < 0.0 ||
                                (now_seconds - live_preview_last_apply_seconds_) >= min_interval_seconds;

    if (!drag_active || throttle_ready) {
      const double apply_start_seconds = GetSteadyTimeSeconds();
      bool applied_any = false;
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
        if (tassel_entities_ptr) {
          const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
          if (drag_active) {
            bool preview_applied = false;
            const uint32_t preview_step_cap = static_cast<uint32_t>(
                std::max(1, live_preview_max_growth_steps));
            const float preview_target_cap = std::max(0.0f, live_preview_max_gdd);

            for (const auto& entity : tassel_entities) {
              if (!scene->IsEntityValid(entity))
                continue;
              auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
              if (!tassel)
                continue;
              if (tassel->descriptor_ref.Get<MaizeTasselDescriptor>().get() != this)
                continue;

              const float preview_target_gdd = live_preview_cap_target_gdd
                  ? std::min(tassel->target_gdd, preview_target_cap)
                  : tassel->target_gdd;
              tassel->GeneratePreviewGeometryEntities(preview_target_gdd, preview_step_cap);
              preview_applied = true;
              applied_any = true;

              if (live_preview_representative_only)
                break;
            }

            if (preview_applied) {
              live_preview_needs_full_apply_ = true;
            }
          } else {
            for (const auto& entity : tassel_entities) {
              if (!scene->IsEntityValid(entity))
                continue;
              auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
              if (!tassel)
                continue;
              if (tassel->descriptor_ref.Get<MaizeTasselDescriptor>().get() != this)
                continue;

              tassel->GenerateGeometryEntities(true);
              applied_any = true;
            }
            live_preview_needs_full_apply_ = false;
          }
        }
      }

      const double apply_end_seconds = GetSteadyTimeSeconds();
      live_preview_last_apply_seconds_ = apply_end_seconds;
      live_preview_last_apply_ms_ = (apply_end_seconds - apply_start_seconds) * 1000.0;
      if (applied_any) {
        live_preview_total_apply_ms_ += live_preview_last_apply_ms_;
        live_preview_apply_count_++;
      }
      live_preview_dirty_ = false;
    } else {
      live_preview_coalesced_count_++;
    }
  }

  return changed || editor_preferences_changed;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

void MaizeTasselDescriptor::Serialize(YAML::Emitter& out) const {
  // Branch zone.
  branch_node_count.Save("branch_node_count", out);
  branch_internode_length.Save("branch_internode_length", out);
  branch_internode_thickness.Save("branch_internode_thickness", out);
  lateral_insertion_angle.Save("lateral_insertion_angle", out);
  lateral_internode_length.Save("lateral_internode_length", out);
  lateral_node_count.Save("lateral_node_count", out);
  peduncle_branch_probability.Save("peduncle_branch_probability", out);

  // Central spike.
  spike_node_count.Save("spike_node_count", out);
  spike_internode_length.Save("spike_internode_length", out);
  spike_internode_thickness.Save("spike_internode_thickness", out);
  spike_zone_branch_probability.Save("spike_zone_branch_probability", out);

  // Main-rachis pair morphology.
  main_pair_proximal_scale_x.Save("main_pair_proximal_scale_x", out);
  main_pair_proximal_scale_y.Save("main_pair_proximal_scale_y", out);
  main_pair_proximal_scale_z.Save("main_pair_proximal_scale_z", out);
  main_pair_proximal_angle.Save("main_pair_proximal_angle", out);
  main_pair_internode_length.Save("main_pair_internode_length", out);
  main_pair_internode_thickness.Save("main_pair_internode_thickness", out);
  main_pair_internode_angle.Save("main_pair_internode_angle", out);
  main_pair_distal_scale_x.Save("main_pair_distal_scale_x", out);
  main_pair_distal_scale_y.Save("main_pair_distal_scale_y", out);
  main_pair_distal_scale_z.Save("main_pair_distal_scale_z", out);
  main_pair_distal_angle.Save("main_pair_distal_angle", out);

  // Non-main-axis pair morphology.
  branch_pair_proximal_scale_x.Save("branch_pair_proximal_scale_x", out);
  branch_pair_proximal_scale_y.Save("branch_pair_proximal_scale_y", out);
  branch_pair_proximal_scale_z.Save("branch_pair_proximal_scale_z", out);
  branch_pair_proximal_angle.Save("branch_pair_proximal_angle", out);
  branch_pair_internode_length.Save("branch_pair_internode_length", out);
  branch_pair_internode_thickness.Save("branch_pair_internode_thickness", out);
  branch_pair_internode_angle.Save("branch_pair_internode_angle", out);
  branch_pair_distal_scale_x.Save("branch_pair_distal_scale_x", out);
  branch_pair_distal_scale_y.Save("branch_pair_distal_scale_y", out);
  branch_pair_distal_scale_z.Save("branch_pair_distal_scale_z", out);
  branch_pair_distal_angle.Save("branch_pair_distal_angle", out);

  // Branch timing and probabilities.
  lateral_initiation_delay_gdd.Save("lateral_initiation_delay_gdd", out);
  spike_anthesis_offset_gdd.Save("spike_anthesis_offset_gdd", out);
  primary_lateral_branch_probability.Save("primary_lateral_branch_probability", out);
  secondary_lateral_branch_probability.Save("secondary_lateral_branch_probability", out);

  // Shared.
  phyllotaxis_angle.Save("phyllotaxis_angle", out);
  branch_azimuth_offset.Save("branch_azimuth_offset", out);
  lateral_thickness_ratio.Save("lateral_thickness_ratio", out);
  final_age_gdd.Save("final_age_gdd", out);

  secondary_insertion_angle.Save("secondary_insertion_angle", out);
  secondary_internode_length.Save("secondary_internode_length", out);
  secondary_internode_thickness.Save("secondary_internode_thickness", out);
  secondary_node_count.Save("secondary_node_count", out);

  rachis_elongation_curve.Save("rachis_elongation_curve", out);
  rachis_thickness_curve.Save("rachis_thickness_curve", out);
  lateral_elongation_curve.Save("lateral_elongation_curve", out);
  lateral_thickness_curve.Save("lateral_thickness_curve", out);
  lateral_angle_development_curve.Save("lateral_angle_development_curve", out);
  pair_proximal_scale_curve.Save("pair_proximal_scale_curve", out);
  pair_proximal_angle_curve.Save("pair_proximal_angle_curve", out);
  pair_internode_length_curve.Save("pair_internode_length_curve", out);
  pair_internode_thickness_curve.Save("pair_internode_thickness_curve", out);
  pair_internode_angle_curve.Save("pair_internode_angle_curve", out);
  pair_distal_scale_curve.Save("pair_distal_scale_curve", out);
  pair_distal_angle_curve.Save("pair_distal_angle_curve", out);

  out << YAML::Key << "base_temperature" << YAML::Value << base_temperature;
  out << YAML::Key << "plastochron_gdd" << YAML::Value << plastochron_gdd;
  out << YAML::Key << "anthesis_gdd" << YAML::Value << anthesis_gdd;
  out << YAML::Key << "max_gdd" << YAML::Value << maturity_gdd;

  out << YAML::Key << "live_preview" << YAML::Value << live_preview;
  out << YAML::Key << "live_preview_rate_hz" << YAML::Value << live_preview_rate_hz;
  out << YAML::Key << "live_preview_representative_only" << YAML::Value
      << live_preview_representative_only;
  out << YAML::Key << "live_preview_cap_target_gdd" << YAML::Value
      << live_preview_cap_target_gdd;
  out << YAML::Key << "live_preview_max_gdd" << YAML::Value << live_preview_max_gdd;
  out << YAML::Key << "live_preview_max_growth_steps" << YAML::Value
      << live_preview_max_growth_steps;
  out << YAML::Key << "grid_rows" << YAML::Value << grid_rows;
  out << YAML::Key << "grid_cols" << YAML::Value << grid_cols;
  out << YAML::Key << "grid_spacing" << YAML::Value << grid_spacing;

  // Explorer preferences.
  out << YAML::Key << "explorer_mode" << YAML::Value << static_cast<int>(explorer_.mode);
  out << YAML::Key << "explorer_speed" << YAML::Value << explorer_.speed;

  // Tropism array.
  out << YAML::Key << "tropism_count" << YAML::Value << static_cast<int>(tropisms.size());
  for (int i = 0; i < static_cast<int>(tropisms.size()); i++) {
    const std::string prefix = "tropism_" + std::to_string(i) + "_";
    tropisms[i].direction_x.Save(prefix + "dir_x", out);
    tropisms[i].direction_y.Save(prefix + "dir_y", out);
    tropisms[i].direction_z.Save(prefix + "dir_z", out);
    tropisms[i].strength.Save(prefix + "strength", out);
    out << YAML::Key << prefix + "usage_chance_percent" << YAML::Value
        << std::clamp(tropisms[i].usage_chance_percent, 0.0f, 100.0f);
    tropisms[i].order_response.Save(prefix + "order_response", out);
  }
}

void MaizeTasselDescriptor::Deserialize(const YAML::Node& in) {
  // Branch zone.
  branch_node_count.Load("branch_node_count", in);
  branch_internode_length.Load("branch_internode_length", in);
  branch_internode_thickness.Load("branch_internode_thickness", in);
  lateral_insertion_angle.Load("lateral_insertion_angle", in);
  lateral_internode_length.Load("lateral_internode_length", in);
  lateral_node_count.Load("lateral_node_count", in);
  peduncle_branch_probability.Load("peduncle_branch_probability", in);
  if (in["primary_branch_probability"]) {
    peduncle_branch_probability.Load("primary_branch_probability", in);
  }

  // Central spike.
  spike_node_count.Load("spike_node_count", in);
  spike_internode_length.Load("spike_internode_length", in);
  spike_internode_thickness.Load("spike_internode_thickness", in);
  spike_zone_branch_probability.Load("spike_zone_branch_probability", in);
  if (in["primary_branch_probability"]) {
    spike_zone_branch_probability.Load("primary_branch_probability", in);
  }

  // Main-rachis pair morphology.
  main_pair_proximal_scale_x.Load("main_pair_proximal_scale_x", in);
  main_pair_proximal_scale_y.Load("main_pair_proximal_scale_y", in);
  main_pair_proximal_scale_z.Load("main_pair_proximal_scale_z", in);
  main_pair_proximal_angle.Load("main_pair_proximal_angle", in);
  main_pair_internode_length.Load("main_pair_internode_length", in);
  main_pair_internode_thickness.Load("main_pair_internode_thickness", in);
  main_pair_internode_angle.Load("main_pair_internode_angle", in);
  main_pair_distal_scale_x.Load("main_pair_distal_scale_x", in);
  main_pair_distal_scale_y.Load("main_pair_distal_scale_y", in);
  main_pair_distal_scale_z.Load("main_pair_distal_scale_z", in);
  main_pair_distal_angle.Load("main_pair_distal_angle", in);

  // Non-main-axis pair morphology.
  branch_pair_proximal_scale_x.Load("branch_pair_proximal_scale_x", in);
  branch_pair_proximal_scale_y.Load("branch_pair_proximal_scale_y", in);
  branch_pair_proximal_scale_z.Load("branch_pair_proximal_scale_z", in);
  branch_pair_proximal_angle.Load("branch_pair_proximal_angle", in);
  branch_pair_internode_length.Load("branch_pair_internode_length", in);
  branch_pair_internode_thickness.Load("branch_pair_internode_thickness", in);
  branch_pair_internode_angle.Load("branch_pair_internode_angle", in);
  branch_pair_distal_scale_x.Load("branch_pair_distal_scale_x", in);
  branch_pair_distal_scale_y.Load("branch_pair_distal_scale_y", in);
  branch_pair_distal_scale_z.Load("branch_pair_distal_scale_z", in);
  branch_pair_distal_angle.Load("branch_pair_distal_angle", in);

  // Branch timing and branch probabilities.
  lateral_initiation_delay_gdd.Load("lateral_initiation_delay_gdd", in);
  spike_anthesis_offset_gdd.Load("spike_anthesis_offset_gdd", in);
  primary_lateral_branch_probability.Load("primary_lateral_branch_probability", in);
  secondary_lateral_branch_probability.Load("secondary_lateral_branch_probability", in);
  if (in["secondary_probability"]) {
    primary_lateral_branch_probability.Load("secondary_probability", in);
    secondary_lateral_branch_probability.Load("secondary_probability", in);
  }

  // Shared.
  phyllotaxis_angle.Load("phyllotaxis_angle", in);
  branch_azimuth_offset.Load("branch_azimuth_offset", in);
  lateral_thickness_ratio.Load("lateral_thickness_ratio", in);
  final_age_gdd.Load("final_age_gdd", in);

  secondary_insertion_angle.Load("secondary_insertion_angle", in);
  secondary_internode_length.Load("secondary_internode_length", in);
  secondary_internode_thickness.Load("secondary_internode_thickness", in);
  secondary_node_count.Load("secondary_node_count", in);

  rachis_elongation_curve.Load("rachis_elongation_curve", in);
  rachis_thickness_curve.Load("rachis_thickness_curve", in);
  lateral_elongation_curve.Load("lateral_elongation_curve", in);
  lateral_thickness_curve.Load("lateral_thickness_curve", in);
  lateral_angle_development_curve.Load("lateral_angle_development_curve", in);
  pair_proximal_scale_curve.Load("pair_proximal_scale_curve", in);
  pair_proximal_angle_curve.Load("pair_proximal_angle_curve", in);
  pair_internode_length_curve.Load("pair_internode_length_curve", in);
  pair_internode_thickness_curve.Load("pair_internode_thickness_curve", in);
  pair_internode_angle_curve.Load("pair_internode_angle_curve", in);
  pair_distal_scale_curve.Load("pair_distal_scale_curve", in);
  pair_distal_angle_curve.Load("pair_distal_angle_curve", in);

  // Legacy growth-curve mappings.
  if (in["spikelet_scale_curve"]) {
    pair_proximal_scale_curve.Load("spikelet_scale_curve", in);
    pair_distal_scale_curve.Load("spikelet_scale_curve", in);
  }
  if (in["spikelet_pedicel_curve"]) {
    pair_internode_length_curve.Load("spikelet_pedicel_curve", in);
    pair_internode_thickness_curve.Load("spikelet_pedicel_curve", in);
  }
  if (in["spikelet_outward_curve"]) {
    pair_proximal_angle_curve.Load("spikelet_outward_curve", in);
    pair_internode_angle_curve.Load("spikelet_outward_curve", in);
    pair_distal_angle_curve.Load("spikelet_outward_curve", in);
  }

  // Legacy fallback mappings.
  if (in["spike_spikelet_scale"]) {
    main_pair_proximal_scale_x.Load("spike_spikelet_scale", in);
    main_pair_proximal_scale_y.Load("spike_spikelet_scale", in);
    main_pair_proximal_scale_z.Load("spike_spikelet_scale", in);
    main_pair_distal_scale_x.Load("spike_spikelet_scale", in);
    main_pair_distal_scale_y.Load("spike_spikelet_scale", in);
    main_pair_distal_scale_z.Load("spike_spikelet_scale", in);
  }
  if (in["lateral_spikelet_scale"]) {
    branch_pair_proximal_scale_x.Load("lateral_spikelet_scale", in);
    branch_pair_proximal_scale_y.Load("lateral_spikelet_scale", in);
    branch_pair_proximal_scale_z.Load("lateral_spikelet_scale", in);
    branch_pair_distal_scale_x.Load("lateral_spikelet_scale", in);
    branch_pair_distal_scale_y.Load("lateral_spikelet_scale", in);
    branch_pair_distal_scale_z.Load("lateral_spikelet_scale", in);
  }
  if (in["spike_pedicel_length"]) {
    main_pair_internode_length.Load("spike_pedicel_length", in);
  }
  if (in["lateral_pedicel_length"]) {
    branch_pair_internode_length.Load("lateral_pedicel_length", in);
  }
  if (in["spikelet_outward_angle"]) {
    main_pair_proximal_angle.Load("spikelet_outward_angle", in);
    main_pair_internode_angle.Load("spikelet_outward_angle", in);
    main_pair_distal_angle.Load("spikelet_outward_angle", in);
    branch_pair_proximal_angle.Load("spikelet_outward_angle", in);
    branch_pair_internode_angle.Load("spikelet_outward_angle", in);
    branch_pair_distal_angle.Load("spikelet_outward_angle", in);
  }

  if (in["breakage_threshold"]) {
    // Legacy compatibility: keep loading old assets that still contain this key.
  }

  if (in["base_temperature"]) base_temperature = in["base_temperature"].as<float>();
  if (in["plastochron_gdd"]) plastochron_gdd = in["plastochron_gdd"].as<float>();
  if (in["anthesis_gdd"]) anthesis_gdd = in["anthesis_gdd"].as<float>();
  if (in["max_gdd"])
    maturity_gdd = in["max_gdd"].as<float>();
  else if (in["maturity_gdd"])
    maturity_gdd = in["maturity_gdd"].as<float>();

  if (in["live_preview"]) live_preview = in["live_preview"].as<bool>();
  if (in["live_preview_rate_hz"]) live_preview_rate_hz = in["live_preview_rate_hz"].as<float>();
  live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
  if (in["live_preview_representative_only"]) {
    live_preview_representative_only = in["live_preview_representative_only"].as<bool>();
  }
  if (in["live_preview_cap_target_gdd"]) {
    live_preview_cap_target_gdd = in["live_preview_cap_target_gdd"].as<bool>();
  }
  if (in["live_preview_max_gdd"]) {
    live_preview_max_gdd = in["live_preview_max_gdd"].as<float>();
  }
  live_preview_max_gdd = std::max(0.0f, live_preview_max_gdd);
  if (in["live_preview_max_growth_steps"]) {
    live_preview_max_growth_steps = in["live_preview_max_growth_steps"].as<int>();
  }
  live_preview_max_growth_steps = std::clamp(live_preview_max_growth_steps, 1, 10000);
  if (in["grid_rows"]) grid_rows = in["grid_rows"].as<int>();
  if (in["grid_cols"]) grid_cols = in["grid_cols"].as<int>();
  if (in["grid_spacing"]) grid_spacing = in["grid_spacing"].as<float>();

  // Explorer preferences.
  if (in["explorer_mode"]) {
    int m = in["explorer_mode"].as<int>();
    if (m >= 0 && m <= 3) explorer_.mode = static_cast<ParamMotionMode>(m);
  }
  if (in["explorer_speed"]) {
    explorer_.speed = std::clamp(in["explorer_speed"].as<float>(), 0.01f, 10.0f);
  }

  // Tropism array.
  tropisms.clear();
  if (in["tropism_count"]) {
    const int count = in["tropism_count"].as<int>();
    for (int i = 0; i < count; i++) {
      TropismEntry entry;
      const std::string prefix = "tropism_" + std::to_string(i) + "_";
      entry.direction_x.Load(prefix + "dir_x", in);
      entry.direction_y.Load(prefix + "dir_y", in);
      entry.direction_z.Load(prefix + "dir_z", in);
      entry.strength.Load(prefix + "strength", in);
      if (in[prefix + "usage_chance_percent"]) {
        entry.usage_chance_percent =
            std::clamp(in[prefix + "usage_chance_percent"].as<float>(), 0.0f, 100.0f);
      }
      entry.order_response.Load(prefix + "order_response", in);
      tropisms.push_back(std::move(entry));
    }
  }
}
