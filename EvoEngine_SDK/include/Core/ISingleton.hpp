#pragma once
namespace evo_engine
{
template <typename T> class ISingleton
{
  protected:
    ISingleton() = default;
    ISingleton(ISingleton &&) = default;
    ISingleton(const ISingleton &) = default;
    ISingleton &operator=(ISingleton &&) = default;
    ISingleton &operator=(const ISingleton &) = default;

  public:
    static T &GetInstance()
    {
        static T instance;
        return instance;
    }
};
} // namespace evo_engine