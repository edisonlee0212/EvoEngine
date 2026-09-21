#include "RuntimeGuiLayer.hpp"
#include <fstream>
#include "Application.hpp"
#include "Camera.hpp"
#include "IRuntimeGui.hpp"
#include "ProjectManager.hpp"
#include "RuntimeGui.hpp"
#include "RuntimePaths.hpp"
#include "Scene.hpp"
#ifdef _WIN32
#  include <Windows.h>
#endif

using namespace evo_engine;

void RuntimeGuiLayer::DrawView(const std::shared_ptr<Camera>& camera, const ImVec2 origin, const ImVec2 size) {
  auto& app = GetApplication();
  const auto epoch = scene_epoch_;
  const auto scene = app.GetActiveScene();
  if (!scene || !camera || size.x < 32 || size.y < 32 || !camera->IsEnabled() ||
      !scene->IsEntityValid(camera->GetOwner()) || !scene->IsEntityEnabled(camera->GetOwner()) ||
      scene->main_camera.Get<Camera>() != camera)
    return;
  std::vector<std::shared_ptr<RuntimeGui>> components;
  for (const auto entity : scene->GetPrivateComponentOwnersList<RuntimeGui>())
    if (auto component = std::dynamic_pointer_cast<RuntimeGui>(scene->GetPrivateComponent(entity, "RuntimeGui").lock()))
      components.push_back(std::move(component));
  std::sort(components.begin(), components.end(), [&](const auto& a, const auto& b) {
    return a->draw_order != b->draw_order
               ? a->draw_order < b->draw_order
               : scene->GetEntityHandle(a->GetOwner()) < scene->GetEntityHandle(b->GetOwner());
  });
  if (components.empty())
    return;
  renderer_.BeginView(origin, size);
  auto& context = renderer_.GetContext();
  for (const auto& component : components) {
    const auto eligible = [&] {
      return epoch == scene_epoch_ && app.GetActiveScene() == scene && scene->IsEntityValid(component->GetOwner()) &&
             scene->IsEntityEnabled(component->GetOwner()) && component->IsEnabled() &&
             scene->GetPrivateComponent(component->GetOwner(), "RuntimeGui").lock() == component &&
             component->camera.Get<Camera>() == camera && scene->main_camera.Get<Camera>() == camera &&
             camera->IsEnabled();
    };
    if (!eligible())
      continue;
    const auto owner = std::to_string(scene->GetHandle()) + "/" +
                       std::to_string(scene->GetEntityHandle(component->GetOwner())) + "/" +
                       std::to_string(component->GetHandle());
    auto [layout, inserted] = loaded_layouts_.try_emplace(owner, component->GetLayout());
    if (inserted && app.GetApplicationInfo().application_mode == ApplicationMode::Player) {
      const auto source = source_scene_.lock();
      const auto entity = scene->GetEntityHandle(component->GetOwner()).GetValue();
      const auto counterpart = source_components_.find(entity);
      if (source && counterpart != source_components_.end())
        if (const auto original = counterpart->second.lock();
            original && original->GetHandle() == component->GetHandle()) {
          const auto key = std::to_string(source->GetHandle()) + "/" + std::to_string(entity) + "/RuntimeGui";
          if (const auto saved = runtime_overrides_.find(key); saved != runtime_overrides_.end())
            component->SetLayout(saved->second);
        }
    }
    if (inserted || layout->second != component->GetLayout())
      context.LoadLayout(owner, component->GetLayout());
    const auto assets = component->GetGuiAssets();
    context.SetInvocation(component, scene, camera);
    for (auto reference : assets) {
      if (!eligible())
        break;
      std::shared_ptr<IRuntimeGui> asset;
      try {
        asset = reference.Get<IRuntimeGui>();
      } catch (const std::exception&) {
        continue;
      }
      if (asset) {
        context.SetOwner(owner, std::to_string(reference.GetAssetHandle()));
        asset->OnGui(context);
      }
    }
    context.SetInvocation({}, {}, {});
    if (eligible()) {
      StoreLayout(component, context.SaveLayout(owner));
      layout->second = component->GetLayout();
    }
  }
  renderer_.FinishView();
}

void RuntimeGuiLayer::OnRuntimeStart() {
  const auto& settings = GetApplication().GetApplicationInfo();
  if (settings.application_mode != ApplicationMode::Player || settings.runtime_gui_layout_revision.empty())
    return;
  const auto source = ProjectManager::GetStartScene().lock();
  source_scene_ = source;
  source_components_.clear();
  if (source)
    for (const auto entity : source->GetPrivateComponentOwnersList<RuntimeGui>())
      source_components_[source->GetEntityHandle(entity).GetValue()] =
          std::dynamic_pointer_cast<RuntimeGui>(source->GetPrivateComponent(entity, "RuntimeGui").lock());
  runtime_overrides_.clear();
  try {
    const auto file = runtime_paths::Resolve("UserData/RuntimeGuiLayouts.json");
    if (!std::filesystem::exists(file))
      return;
    const auto data = YAML::LoadFile(file.string());
    if (data["revision"].as<std::string>() != settings.runtime_gui_layout_revision)
      return;
    if (const auto layouts = data["layouts"]; layouts && layouts.IsMap())
      for (const auto& entry : layouts)
        runtime_overrides_[entry.first.as<std::string>()] = entry.second.as<std::string>();
  } catch (const std::exception& error) {
    EVOENGINE_WARNING(std::string("Cannot read runtime GUI layouts: ") + error.what());
  }
}

void RuntimeGuiLayer::StoreLayout(const std::shared_ptr<RuntimeGui>& component, std::string layout) {
  const auto& app = GetApplication();
  const auto scene = component->GetScene();
  if (component->GetLayout() != layout) {
    component->SetLayout(std::move(layout));
    if (scene && app.GetApplicationInfo().application_mode == ApplicationMode::Editor &&
        app.GetApplicationStatus() == Application::ExecutionStatus::NotPlaying)
      scene->SetUnsaved();
  }
  if (app.GetApplicationInfo().application_mode != ApplicationMode::Player ||
      app.GetApplicationInfo().runtime_gui_layout_revision.empty())
    return;
  const auto source = source_scene_.lock();
  if (!source || !scene || source == scene || !scene->IsEntityValid(component->GetOwner()))
    return;
  const auto entity = scene->GetEntityHandle(component->GetOwner()).GetValue();
  const auto found = source_components_.find(entity);
  if (found == source_components_.end())
    return;
  const auto original = found->second.lock();
  if (!original || original->GetHandle() != component->GetHandle() || !source->IsEntityValid(original->GetOwner()) ||
      source->GetPrivateComponent(original->GetOwner(), "RuntimeGui").lock() != original)
    return;
  const auto key = std::to_string(source->GetHandle()) + "/" + std::to_string(entity) + "/RuntimeGui";
  if (runtime_overrides_[key] != component->GetLayout()) {
    runtime_overrides_[key] = component->GetLayout();
    overrides_dirty_ = true;
  }
}

void RuntimeGuiLayer::SaveOverrides() {
  if (!overrides_dirty_)
    return;
  const auto& revision = GetApplication().GetApplicationInfo().runtime_gui_layout_revision;
  try {
    const auto file = runtime_paths::Resolve("UserData/RuntimeGuiLayouts.json");
    const auto temporary = file.parent_path() / "RuntimeGuiLayouts.json.tmp";
    std::filesystem::create_directories(file.parent_path());
    YAML::Emitter out;
    out.SetMapFormat(YAML::Flow);
    out.SetStringFormat(YAML::DoubleQuoted);
    out << YAML::BeginMap << YAML::Key << "revision" << YAML::Value << revision;
    out << YAML::Key << "layouts" << YAML::Value << YAML::BeginMap;
    for (const auto& [key, value] : runtime_overrides_)
      out << YAML::Key << key << YAML::Value << value;
    out << YAML::EndMap << YAML::EndMap;
    {
      std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
      stream << out.c_str();
      stream.flush();
      if (!stream)
        throw std::runtime_error("Cannot write layout file.");
    }
#ifdef _WIN32
    if (!MoveFileExW(temporary.c_str(), file.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
      throw std::runtime_error("Cannot replace layout file.");
#else
    std::filesystem::rename(temporary, file);
#endif
    overrides_dirty_ = false;
  } catch (const std::exception& error) {
    EVOENGINE_WARNING(std::string("Cannot save runtime GUI layouts: ") + error.what());
  }
}

void RuntimeGuiLayer::FlushLayouts() {
  const auto scene = GetApplication().GetActiveScene();
  if (!scene || !ImGui::GetCurrentContext())
    return;
  for (const auto entity : scene->GetPrivateComponentOwnersList<RuntimeGui>())
    if (const auto component =
            std::dynamic_pointer_cast<RuntimeGui>(scene->GetPrivateComponent(entity, "RuntimeGui").lock())) {
      const auto owner = std::to_string(scene->GetHandle()) + "/" + std::to_string(scene->GetEntityHandle(entity)) +
                         "/" + std::to_string(component->GetHandle());
      if (loaded_layouts_.find(owner) == loaded_layouts_.end())
        continue;
      StoreLayout(component, renderer_.GetContext().SaveLayout(owner));
    }
}

void RuntimeGuiLayer::OnBeforeSceneDetach() {
  ++scene_epoch_;
  FlushLayouts();
  SaveOverrides();
  loaded_layouts_.clear();
  renderer_.GetContext().ResetLayouts();
}
