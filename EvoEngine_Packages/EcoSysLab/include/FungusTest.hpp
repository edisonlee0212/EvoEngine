#pragma once

namespace eco_sys_lab_package {
using namespace evo_engine;

struct Point3D {
  float x;
  float y;
  float z;
};

struct Edge {
  int source;
  int target;
};

struct Graph {
  std::vector<Point3D> nodes;
  std::vector<Edge> edges;
};

class FungusTest : public IPrivateComponent {
 public:
  Graph Tree_Graph;
  int num_nodes = 3750;
  float a = 0.05f;
  float b = 0.1f;
  float c = 0.05f;
  std::vector<float> C;
  std::vector<float> HC;
  std::vector<float> HL;
  std::vector<float> RW;
  std::vector<float> RB;
  std::vector<float> K;

  std::vector<std::vector<float>> P;

  std::vector<std::vector<float>> DC;
  std::vector<std::vector<float>> DW;
  std::vector<std::vector<float>> DB;

  bool update;
  float aw;
  float ab;
  float bw;
  float bb;
  float ycw;
  float ycb;
  float ylw;
  float pc;
  float pl;
  float k;
  float delta;
  float ll;
  float lc;

  float dt = 0.001f;      // Time step for simulation
  void Initialize();      // Initialize the fungus model
  void ExplicitUpdate();  // Update using explicit scheme

  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  void FixedUpdate() override;

  void OnCreate() override;

  bool inEllipsoid(const Point3D& p, const Point3D& q, float a, float b, float c);

 private:
  struct GpuResources {
    std::shared_ptr<DescriptorSetLayout> descriptor_set_layout;
    std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets;

    std::shared_ptr<ComputePipeline> compute_pipeline;

    std::shared_ptr<Buffer> c_buffer;
    std::shared_ptr<Buffer> hc_buffer;
    std::shared_ptr<Buffer> hl_buffer;
    std::shared_ptr<Buffer> rw_buffer;
    std::shared_ptr<Buffer> rb_buffer;
    std::shared_ptr<Buffer> k_buffer;

    std::shared_ptr<Buffer> c_pre_buffer;
    std::shared_ptr<Buffer> hc_pre_buffer;
    std::shared_ptr<Buffer> hl_pre_buffer;
    std::shared_ptr<Buffer> rw_pre_buffer;
    std::shared_ptr<Buffer> rb_pre_buffer;

    std::shared_ptr<Buffer> edge_buffer;
    std::shared_ptr<Buffer> diffusion_c_buffer;
    std::shared_ptr<Buffer> diffusion_w_buffer;
    std::shared_ptr<Buffer> diffusion_b_buffer;

    std::shared_ptr<Buffer> node_position_buffer;
    std::shared_ptr<Buffer> node_distance_buffer;

    bool initialized = false;
  };

  GpuResources gpu_resources;

  void InitializeGpuResources();
  void DownloadGpuResults();
};
}  // namespace eco_sys_lab_package