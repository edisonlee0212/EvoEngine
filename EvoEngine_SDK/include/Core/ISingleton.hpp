#pragma once
namespace evo_engine {
#define EVOENGINE_SINGLETON_INSTANCE(TYPE) \
 public:                                   \
  static TYPE &GetInstance() {             \
    static TYPE instance;                  \
    return instance;                       \
  }                                        \
                                           \
 private:
}  // namespace evo_engine