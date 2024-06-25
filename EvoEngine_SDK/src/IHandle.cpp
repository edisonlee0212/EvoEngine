#include <IHandle.hpp>
using namespace evo_engine;

static std::random_device s_random_device;
static std::mt19937_64 eng(s_random_device());
static std::uniform_int_distribution<uint64_t> s_uniform_distribution;

Handle::Handle() : value_(s_uniform_distribution(eng)) {
}

Handle::Handle(uint64_t value) : value_(value) {
}

Handle::Handle(const Handle &other) : value_(other.value_) {
}