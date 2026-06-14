#include "ISerializable.hpp"
#include "Application.hpp"
#include "Serialization.hpp"
using namespace evo_engine;

Application &ISerializable::GetApplication() const {
  return *application_;
}

void ISerializable::Save(const std::string &name, YAML::Emitter &out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  Serialization::SerializeObject(out, *this);
  out << YAML::EndMap;
}
void ISerializable::Load(const std::string &name, const YAML::Node &in) {
  if (in[name]) {
    const auto &cd = in[name];
    Serialization::DeserializeObject(cd, *this);
  }
}
