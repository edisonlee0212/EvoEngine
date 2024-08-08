#pragma once

#include <memory>
#include <type_traits>
#include <utility>

namespace evo_engine {

/**
 * @brief This class is a simplified implementation of C++17's `std::experimental::propagate_const`
 * for managing a pointer to a private implementation (PImpl) whilst enforcing const-correctness.
 *
 * @tparam T Type of the object owned by PImplPtr.
 * @tparam Deleter Deleter type used for destruction of the owned object, defaults to `std::default_delete<T>`.
 */
template <typename T, class Deleter = std::default_delete<T>>
class PImplPtr {
 public:
  /**
   * @brief Default constructor that initializes a new instance of T.
   */
  PImplPtr() : ptr_(new T) {
  }

  /**
   * @brief Constructs a PImplPtr with a null pointer.
   *
   * @param p A nullptr.
   */
  explicit PImplPtr(nullptr_t) noexcept : ptr_(nullptr) {
  }

  /**
   * @brief Constructs a PImplPtr with raw pointer to T.
   *
   * @param p A raw pointer to an instance of T.
   */
  explicit PImplPtr(T* p) noexcept : ptr_(p) {
  }

  /**
   * @brief Variadic template constructor that forwards arguments to T's constructor.
   *
   * @tparam Args Variadic template parameter pack for constructor arguments.
   * @param args Arguments to forward to the constructor of T.
   */
  template <typename... Args>
  PImplPtr(Args&&... args) : ptr_(new T(std::forward<Args>(args)...)) {
  }

  /**
   * @brief Destructor that deletes the owned object using the specified deleter.
   */
  ~PImplPtr() noexcept {
    Deleter()(ptr_);
  }

  /**
   * @brief Copy constructor that performs a deep copy of the object owned by `other`.
   *
   * @param other Another PImplPtr from which to copy.
   */
  PImplPtr(const PImplPtr& other) : ptr_(other.ptr_ ? new T(*other.ptr_) : nullptr) {
  }

  /**
   * @brief Move constructor that takes ownership of the object owned by `other`.
   *
   * @param other Another PImplPtr to move from.
   */
  PImplPtr(PImplPtr&& other) noexcept : ptr_(std::move(other.ptr_)) {
    other.ptr_ = nullptr;
  }

  /**
   * @brief Copy assignment operator that performs a deep copy of the object owned by `other`.
   *
   * @param other Another PImplPtr to assign from.
   * @return A reference to the PImplPtr after assignment.
   */
  PImplPtr& operator=(const PImplPtr& other) {
    if (this != &other) {
      if (other.ptr_) {
        if (ptr_ == nullptr) {
          ptr_ = new T(*other.ptr_);
        } else {
          *ptr_ = *other.ptr_;
        }
      } else {
        Deleter()(ptr_);
        ptr_ = nullptr;
      }
    }

    return *this;
  }

  /**
   * @brief Move assignment operator that takes ownership of the object owned by `other`.
   *
   * @param other Another PImplPtr to assign from.
   * @return A reference to the PImplPtr after assignment.
   */
  PImplPtr& operator=(PImplPtr&& other) noexcept {
    if (this != &other) {
      Deleter()(ptr_);
      ptr_ = std::move(other.ptr_);
      other.ptr_ = nullptr;
    }

    return *this;
  }

  /**
   * @brief Accessor to the underlying pointer for non-const operations.
   *
   * @return A raw pointer to the owned object of type T.
   */
  T* Get() noexcept {
    return ptr_;
  }

  /**
   * @brief Accessor to the underlying pointer for const operations.
   *
   * @return A const raw pointer to the owned object of type T.
   */
  const T* Get() const noexcept {
    return ptr_;
  }

  /**
   * @brief Overload of the arrow operator for accessing member functions of T.
   *
   * @return A raw pointer to the owned object of type T.
   */
  T* operator->() noexcept {
    return ptr_;
  }

  /**
   * @brief Const overload of the arrow operator for accessing member functions of T.
   *
   * @return A const raw pointer to the owned object of type T.
   */
  const T* operator->() const noexcept {
    return ptr_;
  }

  /**
   * @brief Dereference operator to gain reference access to the owned object.
   *
   * @return A reference to the owned object of type T.
   */
  T& operator*() noexcept {
    return *ptr_;
  }

  /**
   * @brief Const dereference operator to gain const reference access to the owned object.
   *
   * @return A const reference to the owned object of type T.
   */
  const T& operator*() const noexcept {
    return *ptr_;
  }

  /**
   * @brief Conversion operator to bool, checking if the pointer is not null.
   *
   * @return True if the pointer is not null, false otherwise.
   */
  operator bool() const noexcept {
    return ptr_ != nullptr;
  }

 private:
  T* ptr_;  ///< The pointer to the object of type T being managed.
};

template <typename T>
struct InternalType;

template <typename T>
using InternalTypeT = typename InternalType<T>::Type;

/**
 * @brief Get the internal interface of t.
 *
 * @param t Reference to an PublicT object.
 * @return InternalT& Reference to the internal instance.
 */
template <typename PublicT, typename InternalT = std::conditional_t<std::is_const_v<PublicT>,
                                                                    const InternalTypeT<std::remove_const_t<PublicT>>,
                                                                    InternalTypeT<std::remove_const_t<PublicT>>>>
InternalT& GetInternal(PublicT& t) noexcept {
  // This hack works under 3 assumptions.
  //   1. The first member variable of PublicT is a PImplPtr<Impl>,
  //   2. The first member variable of PImplPtr is Impl*, (always true)
  //   3. The Impl is single inherited from InternalT.
  // Then the address of t is a pointer of pointer to the InternalT object. In this way, we can get the
  // InternalT without intrusion to PublicT.
  using InternalTPtr = InternalT*;
  using CastT = std::conditional_t<std::is_const_v<InternalT>, const InternalTPtr, InternalTPtr>;
  return *reinterpret_cast<CastT&>(t);
}
}  // namespace evo_engine
