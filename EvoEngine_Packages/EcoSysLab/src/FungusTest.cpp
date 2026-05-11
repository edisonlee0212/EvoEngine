#include "FungusTest.hpp"
#include <glm/glm.hpp>
#include "Application.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "Times.hpp"
#include "Transform.hpp"

using namespace eco_sys_lab_package;

struct SimulationParams {
  float dt;
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
  int num_nodes;
  int edge_count;
  glm::mat3 matrixAw;
  glm::mat3 matrixAb;
  glm::mat3 matrixAc;
};

bool FungusTest::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Enable Update", &update);
  if (update) {
    DownloadGpuResults();
    update = false;
  }

  static std::shared_ptr<ParticleInfoList> particle_info_list;
  if (!particle_info_list) {
    particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }
  std::vector<ParticleInfo> particle_infos(num_nodes);
  // const auto time = ApplicationContext::Get().GetTimes().Now();
  Jobs::RunParallelFor(particle_infos.size(), [&](size_t i) {
    auto& particle_info = particle_infos[i];
    particle_info.instance_matrix.SetPosition(
        glm::vec3(Tree_Graph.nodes[i].x, Tree_Graph.nodes[i].y, Tree_Graph.nodes[i].z));
    // particle_info.instance_color = glm::vec4(0.55f * (RW[i]),
    //                                          0.7f * (1.f - (RW[i])) + 0.3f,
    //                                          0.15f * (RW[i]),
    //                                          0.5f
    //);
    particle_info.instance_color =
        glm::vec4(0.55f + 0.45f * (1.f - HL[i]), 0.3f + 0.7f * (1.f - HL[i]), 0.15f + 0.85 * (1.f - HL[i]), 1.f);
  });

  particle_info_list->SetParticleInfos(particle_infos);
  GizmoSettings gizmo_settings{};
  gizmo_settings.draw_settings.blending = false;
  gizmo_settings.depth_test = true;
  gizmo_settings.depth_write = true;

  editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().sphere,
                                              editor_layer->GetSceneCamera(), particle_info_list, glm::mat4(1), 0.02f,
                                              gizmo_settings);
  return false;
}

std::string removeChars(const std::string& str, const std::string& charsToRemove) {
  std::string result;
  for (char c : str) {
    if (charsToRemove.find(c) == std::string::npos) {
      result.push_back(c);
    }
  }
  return result;
}

void FungusTest::OnCreate() {
  update = false;
  C.resize(num_nodes, 0.0f);
  HC.resize(num_nodes, 1.0f);
  HL.resize(num_nodes, 1.0f);
  RW.resize(num_nodes, 0.0f);
  RB.resize(num_nodes, 0.0f);
  P.resize(num_nodes, std::vector<float>(3, 0.0f));
  DC.resize(num_nodes, std::vector<float>(num_nodes, 0.0f));
  DW.resize(num_nodes, std::vector<float>(num_nodes, 0.0f));
  DB.resize(num_nodes, std::vector<float>(num_nodes, 0.0f));
  K.resize(num_nodes, 0.0f);
  aw = 5.0f;
  ab = 5.0f;
  bw = 3.0f;
  bb = 3.0f;
  ycw = 1.0f;
  ycb = 1.0f;
  ylw = 2.0f;
  pc = 0.2f;
  pl = 0.1f;
  k = 0.2f;
  delta = 0.05f;
  ll = 0.5f;
  lc = 0.5f;
  // Tree_Graph.nodes.resize(num_nodes);
  // std::random_device rd;
  // std::mt19937 gen(rd());

  // std::uniform_real_distribution<float> disX(0.0f, 0.5f);
  // std::uniform_real_distribution<float> disY(0.0f, 1.0f);
  // std::uniform_real_distribution<float> disZ(0.0f, 0.5f);

  // for (int i = 0; i < num_nodes; ++i) {
  //   Tree_Graph.nodes[i].x = disX(gen);
  //   Tree_Graph.nodes[i].y = disY(gen);
  //   Tree_Graph.nodes[i].z = disZ(gen);
  // }

  // for (int i = 0; i < num_nodes; ++i) {
  //   for (int j = i + 1; j < num_nodes; ++j) {
  //     if (inEllipsoid(Tree_Graph.nodes[i], Tree_Graph.nodes[j], a, b, c) ||
  //         inEllipsoid(Tree_Graph.nodes[j], Tree_Graph.nodes[i], a, b, c)) {
  //       Tree_Graph.edges.push_back({i, j});
  //     }
  //   }
  // }

  // std::ofstream outfile("graph3d_float.txt");
  // if (!outfile) {
  //   std::cerr << "Cannot open files!" << std::endl;
  // }

  // outfile << "Nodes:\n";
  // for (int i = 0; i < static_cast<int>(Tree_Graph.nodes.size()); ++i) {
  //   outfile << i << ": (" << Tree_Graph.nodes[i].x << ", " << Tree_Graph.nodes[i].y << ", " << Tree_Graph.nodes[i].z
  //           << ")\n";
  // }

  // outfile << "Edges:\n";
  // for (const auto& edge : Tree_Graph.edges) {
  //   outfile << edge.source << " -- " << edge.target << "\n";
  // }

  // outfile.close();

  std::ifstream infile("graph3d_float.txt");
  if (!infile) {
    std::cerr << "Cannot open "
              << "graph3d_float.txt"
              << " for reading!" << std::endl;
  }

  std::string line;
  bool readingNodes = false;
  bool readingEdges = false;

  while (std::getline(infile, line)) {
    if (line.empty())
      continue;
    if (line.find("Nodes:") != std::string::npos) {
      readingNodes = true;
      readingEdges = false;
      continue;
    }
    if (line.find("Edges:") != std::string::npos) {
      readingEdges = true;
      readingNodes = false;
      continue;
    }

    if (readingNodes) {
      std::istringstream iss(line);
      int index;
      char colon;
      std::string coordStr;
      if (!(iss >> index >> colon))
        continue;
      std::getline(iss, coordStr);
      coordStr = removeChars(coordStr, " ()");
      for (char& c : coordStr) {
        if (c == ',')
          c = ' ';
      }
      std::istringstream coordStream(coordStr);
      float x, y, z;
      if (coordStream >> x >> y >> z) {
        Tree_Graph.nodes.push_back({x, 1.0f - y, z});
      }
    } else if (readingEdges) {
      std::istringstream iss(line);
      int source, target;
      if (!(iss >> source))
        continue;
      std::string dash;
      iss >> dash;
      iss >> target;
      Tree_Graph.edges.push_back({source, target});
    }
  }
  HL[0] = 0.0;

  Initialize();

  // printf("Created!");
}

void FungusTest::FixedUpdate() {
  // Update the fungus model
  for (int i = 0; i < 10; i++) {
    ExplicitUpdate();
  }
  // ExplicitUpdate();
  // std::cout << RW[0] << "\n" << std::endl;
}

bool FungusTest::inEllipsoid(const Point3D& p, const Point3D& q, float a, float b, float c) {
  float dx = q.x - p.x;
  float dy = q.y - p.y;
  float dz = q.z - p.z;
  return (dx * dx) / (a * a) + (dy * dy) / (b * b) + (dz * dz) / (c * c) <= 1.0;
}

void FungusTest::Initialize() {
  // Fill C with initial carbon value
  std::fill(C.begin(), C.end(), 0.2f);

  // Fill health values
  std::fill(HC.begin(), HC.end(), 1.0f);
  std::fill(HL.begin(), HL.end(), 1.0f);

  // Set initial resources at root node
  RW[0] = 1.0f;
  RB[0] = 1.0f;

  // Extract positions from your nodes
  for (int i = 0; i < num_nodes; ++i) {
    P[i][0] = Tree_Graph.nodes[i].x;
    P[i][1] = Tree_Graph.nodes[i].y;
    P[i][2] = Tree_Graph.nodes[i].z;
    // K[i] = 1.f / (8.f * std::min({P[i][0], P[i][2], 0.5f - P[i][0], 0.5f - P[i][2]}) + 0.25f);
  }
}

void FungusTest::ExplicitUpdate() {
  if (!gpu_resources.initialized) {
    InitializeGpuResources();
  }

  SimulationParams push_constants;

  push_constants.dt = dt;
  push_constants.aw = aw;
  push_constants.ab = ab;
  push_constants.bw = bw;
  push_constants.bb = bb;
  push_constants.ycw = ycw;
  push_constants.ycb = ycb;
  push_constants.ylw = ylw;
  push_constants.pc = pc;
  push_constants.pl = pl;
  push_constants.k = k;
  push_constants.delta = delta;
  push_constants.ll = ll;
  push_constants.lc = lc;
  push_constants.num_nodes = num_nodes;
  push_constants.edge_count = Tree_Graph.edges.size();
  push_constants.matrixAw = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 1.0f);
  push_constants.matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 1.0f);
  push_constants.matrixAc = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.5f);

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    gpu_resources.compute_pipeline->Bind(vk_command_buffer);

    gpu_resources.compute_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, gpu_resources.descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    gpu_resources.compute_pipeline->PushConstant(vk_command_buffer, 0, push_constants);

    gpu_resources.compute_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(num_nodes, work_group_invocations), 1,
                                             1);

    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void FungusTest::DownloadGpuResults() {
  gpu_resources.c_buffer->DownloadVector(C, num_nodes);
  gpu_resources.hc_buffer->DownloadVector(HC, num_nodes);
  gpu_resources.hl_buffer->DownloadVector(HL, num_nodes);
  gpu_resources.rw_buffer->DownloadVector(RW, num_nodes);
  gpu_resources.rb_buffer->DownloadVector(RB, num_nodes);
}

void FungusTest::InitializeGpuResources() {
  if (gpu_resources.initialized)
    return;

  gpu_resources.descriptor_set_layout = std::make_shared<DescriptorSetLayout>();

  for (int i = 0; i < 17; i++) {
    gpu_resources.descriptor_set_layout->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                               VK_SHADER_STAGE_COMPUTE_BIT, 0);
  }

  gpu_resources.descriptor_set_layout->Initialize();

  static std::shared_ptr<Shader> compute_shader{};
  compute_shader = std::make_shared<Shader>();
  compute_shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                             std::filesystem::path("./EcoSysLabResources") /
                                 "Shaders/Compute/DynamicStrands/Fungus/FungusDiffusion_test.comp");

  gpu_resources.compute_pipeline = std::make_shared<ComputePipeline>();
  gpu_resources.compute_pipeline->compute_shader = compute_shader;
  gpu_resources.compute_pipeline->descriptor_set_layouts.emplace_back(gpu_resources.descriptor_set_layout);

  auto& push_constant_range = gpu_resources.compute_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(SimulationParams);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  gpu_resources.compute_pipeline->Initialize();

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo alloc_info{};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  buffer_create_info.size = sizeof(float) * num_nodes;

  gpu_resources.c_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.hc_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.hl_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.rw_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.rb_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.k_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);

  gpu_resources.c_pre_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.hc_pre_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.hl_pre_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.rw_pre_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.rb_pre_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);

  gpu_resources.diffusion_c_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.diffusion_w_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.diffusion_b_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);
  gpu_resources.node_distance_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);

  buffer_create_info.size = sizeof(Edge) * Tree_Graph.edges.size();
  gpu_resources.edge_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);

  buffer_create_info.size = sizeof(glm::vec3) * num_nodes;
  gpu_resources.node_position_buffer = std::make_shared<Buffer>(buffer_create_info, alloc_info);

  gpu_resources.c_buffer->UploadVector(C);
  gpu_resources.hc_buffer->UploadVector(HC);
  gpu_resources.hl_buffer->UploadVector(HL);
  gpu_resources.rw_buffer->UploadVector(RW);
  gpu_resources.rb_buffer->UploadVector(RB);
  gpu_resources.k_buffer->UploadVector(K);

  gpu_resources.c_pre_buffer->UploadVector(C);
  gpu_resources.hc_pre_buffer->UploadVector(HC);
  gpu_resources.hl_pre_buffer->UploadVector(HL);
  gpu_resources.rw_pre_buffer->UploadVector(RW);
  gpu_resources.rb_pre_buffer->UploadVector(RB);

  std::vector<glm::vec3> node_positions(num_nodes);
  std::vector<float> node_distances(num_nodes);

  for (int i = 0; i < num_nodes; i++) {
    node_positions[i] = glm::vec3(P[i][0], P[i][1], P[i][2]);
    float distance_to_boundary = std::min({P[i][0], P[i][2], 0.5f - P[i][0], 0.5f - P[i][2]});
    node_distances[i] = distance_to_boundary;
  }
  gpu_resources.node_position_buffer->UploadVector(node_positions);
  gpu_resources.node_distance_buffer->UploadVector(node_distances);

  std::vector<float> zeros(num_nodes, 0.0f);
  gpu_resources.diffusion_c_buffer->UploadVector(zeros);
  gpu_resources.diffusion_w_buffer->UploadVector(zeros);
  gpu_resources.diffusion_b_buffer->UploadVector(zeros);

  gpu_resources.edge_buffer->UploadVector(Tree_Graph.edges);

  const auto max_frames = Platform::GetMaxFramesInFlight();
  gpu_resources.descriptor_sets.resize(max_frames);

  for (int i = 0; i < max_frames; i++) {
    gpu_resources.descriptor_sets[i] = std::make_shared<DescriptorSet>(gpu_resources.descriptor_set_layout);

    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(0, gpu_resources.c_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(1, gpu_resources.hc_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(2, gpu_resources.hl_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(3, gpu_resources.rw_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(4, gpu_resources.rb_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(5, gpu_resources.k_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(6, gpu_resources.c_pre_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(7, gpu_resources.hc_pre_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(8, gpu_resources.hl_pre_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(9, gpu_resources.rw_pre_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(10, gpu_resources.rb_pre_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(11, gpu_resources.edge_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(12, gpu_resources.diffusion_c_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(13, gpu_resources.diffusion_w_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(14, gpu_resources.diffusion_b_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(15, gpu_resources.node_position_buffer);
    gpu_resources.descriptor_sets[i]->UpdateBufferDescriptorBinding(16, gpu_resources.node_distance_buffer);
  }
  gpu_resources.initialized = true;
}
