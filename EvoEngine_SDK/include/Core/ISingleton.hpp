
#pragma once
namespace evo_engine {

/**
 * @def EVOENGINE_SINGLETON_INSTANCE(TYPE)
 * @brief Macro to create a singleton instance of a given type.
 *
 * This macro provides a default implementation for a singleton pattern
 * by defining a static method `GetInstance()` that returns a single
 * instance of the provided type. Usage of this macro ensures that
 * the class follows the singleton design pattern.
 *
 * Example usage:
 * @code
 * class MyClass {
 *   EVOENGINE_SINGLETON_INSTANCE(MyClass)
 *   ...
 * };
 *
 * auto &instance = MyClass::GetInstance();
 * @endcode
 *
 * @param TYPE The class type for which the singleton instance is defined.
 */
#define EVOENGINE_SINGLETON_INSTANCE(TYPE) \
 public:                                   \
  static TYPE &GetInstance() {             \
    static TYPE instance;                  \
    return instance;                       \
  }                                        \
                                           \
 private:

}  // namespace evo_engine
