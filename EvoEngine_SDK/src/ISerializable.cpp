#include "ISerializable.hpp"
using namespace evo_engine;
void ISerializable::Save(const std::string &name, YAML::Emitter &out) const
{
    out << YAML::Key << name << YAML::Value << YAML::BeginMap;
    {
        Serialize(out);
    }
    out << YAML::EndMap;
}
void ISerializable::Load(const std::string &name, const YAML::Node &in)
{
    if (in[name]) {
        const auto &cd = in[name];
        Deserialize(cd);
    }
}
