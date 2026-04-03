#pragma once

#include <variant>
#include <vector>
#include <string>
#include <cassert>

namespace l_system_plugin {

/**
 * @brief Typed container for module data in the L-system graph.
 *
 * Wraps a std::variant of user-defined module structs. Each module type
 * corresponds to a symbol in the L-system grammar (e.g., Apex, Internode, Leaf).
 *
 * Usage:
 * @code
 *   struct Apex  { float vigor; int order; };
 *   struct Inter { float length; float thickness; };
 *   struct Leaf  { float area; float angle; };
 *
 *   using MyModules = ModuleVariant<Apex, Inter, Leaf>;
 *   // symbol_id 0 = Apex, 1 = Inter, 2 = Leaf (matches variant index)
 *
 *   MyModules m;
 *   m.Set<Apex>({1.0f, 0});  // Sets data and symbol_id automatically
 *   auto& apex = m.Get<Apex>();
 * @endcode
 *
 * @tparam ModuleTypes All module struct types used in this L-system.
 */
template <typename... ModuleTypes>
struct ModuleVariant {
  std::variant<ModuleTypes...> data;

  ModuleVariant() = default;

  /**
   * @brief Get the symbol_id (variant index) of the currently held module type.
   */
  [[nodiscard]] int SymbolId() const {
    return static_cast<int>(data.index());
  }

  /**
   * @brief Set the module data to a specific type. Symbol ID is inferred from the variant index.
   */
  template <typename T>
  void Set(const T& value) {
    data = value;
  }

  /**
   * @brief Set the module data to a specific type via move.
   */
  template <typename T>
  void Set(T&& value) {
    data = std::move(value);
  }

  /**
   * @brief Get a mutable reference to the module data, assuming it holds type T.
   *        UB if the variant doesn't hold T.
   */
  template <typename T>
  [[nodiscard]] T& Get() {
    return std::get<T>(data);
  }

  /**
   * @brief Get a const reference to the module data, assuming it holds type T.
   */
  template <typename T>
  [[nodiscard]] const T& Get() const {
    return std::get<T>(data);
  }

  /**
   * @brief Check if the variant currently holds type T.
   */
  template <typename T>
  [[nodiscard]] bool Is() const {
    return std::holds_alternative<T>(data);
  }

  /**
   * @brief Visit the variant with a callable (visitor pattern).
   */
  template <typename Visitor>
  decltype(auto) Visit(Visitor&& visitor) {
    return std::visit(std::forward<Visitor>(visitor), data);
  }

  template <typename Visitor>
  decltype(auto) Visit(Visitor&& visitor) const {
    return std::visit(std::forward<Visitor>(visitor), data);
  }
};

/**
 * @brief Compile-time helper to get the index of type T within a parameter pack.
 *
 * Usage: constexpr int id = ModuleIndex<Apex, Apex, Inter, Leaf>::value; // 0
 */
template <typename T, typename... Types>
struct ModuleIndex;

// Specialization: T matches the first type in the pack → index is 0.
template <typename T, typename... Rest>
struct ModuleIndex<T, T, Rest...> {
  static constexpr int value = 0;
};

// Specialization: T does not match First → recurse into Rest.
template <typename T, typename First, typename... Rest>
struct ModuleIndex<T, First, Rest...> {
  static constexpr int value = 1 + ModuleIndex<T, Rest...>::value;
};

template <typename T>
struct ModuleIndex<T> {
  static_assert(sizeof(T) == 0, "Type not found in module type list");
};

/**
 * @brief Compile-time count of module types.
 */
template <typename... ModuleTypes>
struct ModuleCount {
  static constexpr int value = sizeof...(ModuleTypes);
};

}  // namespace l_system_plugin
