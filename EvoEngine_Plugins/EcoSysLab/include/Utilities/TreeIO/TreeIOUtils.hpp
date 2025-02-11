/**
 * @author Tomas Polasek
 * @date 11.20.2019
 * @version 1.0
 * @brief Utilities and statistics for the treeio::Tree class.
 */

#ifndef TREEIO_UTILS_H
#define TREEIO_UTILS_H

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <glm/glm.hpp>
#define HAS_GLM

#include "TreeIOVectorT.hpp"

#define TREE_UNUSED(x) (void)(x)

namespace treeutil {

/// @brief Color wrapper.
struct Color {
  constexpr Color(float rv, float gv, float bv, float av = 1.0f) : r{rv}, g{gv}, b{bv}, a{av} {
  }
  constexpr Color() : Color(0.0f, 0.0f, 0.0f, 1.0f) {
  }

  /// @brief Set colors from given array.
  void fromArray(const float (&color)[4]) {
    r = color[0];
    g = color[1];
    b = color[2], a = color[3];
  }

  /// @brief Get pointer to the first value of 4-value float array containing RGBA.
  float *data() {
    return &r;
  }
  /// @brief Get pointer to the first value of 4-value float array containing RGBA.
  const float *data() const {
    return &r;
  }

  /// @brief Create color from given color array.
  static Color createFromArray(const float (&color)[4]) {
    Color c{};
    c.fromArray(color);
    return c;
  }

  float r;
  float g;
  float b;
  float a;
};  // struct Color

/// @brief Specification of 3D bounding box.
struct BoundingBox {
  /// Position of the bounding box corner.
  Vector3D position{0.0f};
  /// Size of the bounding box.
  Vector3D size{0.0f};

  /// Is this bounding box filled with valid values?
  bool filled{false};

  /// @brief Calculate center of the bounding box.
  std::array<float, 3u> center() const;
  /// @brief Calculate diameter of the bounding box.
  float diameter() const;
};  // struct BoundingBox

/// @brief Pointer type used for instance representation.
template <typename PT>
using WrapperPtrT = std::shared_ptr<PT>;
/// @brief Type used to construct instances of the pointer.
template <typename PT, typename... CArgTs>
static auto WrapperCtrT(CArgTs... cArgs) {
  return std::make_shared<PT>(std::forward<CArgTs>(cArgs)...);
};
/// @brief Perform pointer cast from PT to TT.
template <typename TT, typename PT>
static auto WrapperCastT(const WrapperPtrT<PT> &ptr) {
  return std::dynamic_pointer_cast<TT>(ptr);
}

/**
 * @brief Inheritable class which adds shared pointer instantiation system.
 * @tparam T Type of class inheriting this class.
 */
template <typename T>
class PointerWrapper {
 public:
  /// Concrete pointer type for the inheriting class.
  using Ptr = WrapperPtrT<T>;

  /// @brief Instantiate the class, passing arguments to its constructor.
  template <typename... CArgTs>
  static Ptr instantiate(CArgTs... cArgs) {
    return WrapperCtrT<T>(std::forward<CArgTs>(cArgs)...);
  }

 private:
 protected:
};  // class PointerWrapper

/// @brief Smart pointer which automatically creates copies instead of propagating the pointer.
template <typename T, template <typename> typename PtrT>
struct CopyableSmartPtr {
  /// @brief Initialize empty pointer.
  CopyableSmartPtr() = default;
  /// @brief Clean up and destroy.
  ~CopyableSmartPtr() = default;

  /// @brief Initialize from given pointer.
  CopyableSmartPtr(T *pointer);
  /// @brief Initialize from given pointer.
  CopyableSmartPtr(const PtrT<T> &pointer);

  // Copy and move constructors.
  CopyableSmartPtr(const CopyableSmartPtr &other);
  CopyableSmartPtr &operator=(CopyableSmartPtr other);
  CopyableSmartPtr(CopyableSmartPtr &&other);
  CopyableSmartPtr &operator=(CopyableSmartPtr &&other);

  /// @brief Swap content with other smart pointer.
  void swap(CopyableSmartPtr &other);

  /// @brief Swap content with other smart pointer.
  static void swap(CopyableSmartPtr &first, CopyableSmartPtr &second);

  // See-through operation routing.
  // Implicit cast to underlying pointer type.
  operator PtrT<T>();
  operator PtrT<T>() const;

  // Access internal data:
  T &operator*();
  const T &operator*() const;
  T *operator->();
  const T *operator->() const;
  operator bool() const;

  // Assign external data:
  CopyableSmartPtr &operator=(const PtrT<T> &other);

  /// Internal pointer.
  PtrT<T> ptr{nullptr};
};  // struct CopyableSharedPtr

template <typename InputItT, typename LambdaT>
class LambdaIterator : public InputItT {
 public:
  /// @brief Initialize the lambda iterator.
  LambdaIterator(const InputItT &iterator, const LambdaT &lambda);
  /// @brief Clean up and destroy.
  ~LambdaIterator();

  /// @brief Apply lambda on *iterator and return.
  auto operator*();
  /// @brief Apply lambda on *iterator and return.
  auto operator*() const;
  /// @brief Apply lambda on iterator-> and return.
  auto operator->();
  /// @brief Apply lambda on iterator-> and return.
  auto operator->() const;
  /// @brief Apply lambda on iterator[] and return.
  auto operator[](std::size_t n);
  /// @brief Apply lambda on iterator[] and return.
  auto operator[](std::size_t n) const;

  // Provide return type information using provided transformation lambda.
  using value_type = typename std::remove_const<typename std::remove_reference<decltype(std::declval<LambdaT>()(
      std::declval<InputItT>().operator*()))>::type>::type;
  using reference = value_type &;
  using pointer = value_type *;

 private:
  /// Transformation function to be called on each access.
  LambdaT mLambda{};

 protected:
};  // class LambdaIterator

/// @brief Convert given string to lower-case.
std::string strToLower(const std::string &str);

/// @brief Generate random real number from 0.0 to 1.0 .
template <typename T = double>
T uniformZeroToOne();

/// @brief Calculate sgn(val) in {T(-1), T(0), T(1)}.
template <typename T>
T sgn(const T &val);

/// @brief Get min and max for given container.
template <typename ArrT>
auto minMax(const ArrT &arr);

/// @brief Get min and argmin for given container. In case of duplicates, the last index will be reported.
template <typename ArrT>
auto argMin(const ArrT &arr);

/// @brief Get max and argmax for given container. In case of duplicates, the last index will be reported.
template <typename ArrT>
auto argMax(const ArrT &arr);

/// @brief Get min, argmin, max and argmax for given container. In case of duplicates, the last index will be reported.
template <typename ArrT>
auto argMinMax(const ArrT &arr);

/// @brief Simple logging manager.
class Logger {
 public:
  /// @brief Logging levels.
  enum Level {
    /// Debugging messages.
    Debug = 0u,
    /// Non-critical information messages.
    Info = 1u,
    /// Warning messages.
    Warning = 2u,
    /// Critical error messages.
    Error = 3u
  };  // enum class Level
  /// Number of logging levels.
  static constexpr auto LOG_LEVEL_COUNT{4u};
  /// Default logging level used when no other is specified.
  static constexpr auto DEFAULT_LOGGING_LEVEL{Level::Info};

  // Fully static class.
  Logger() = delete;

  /// @brief Get logging stream for given logging level.
  static inline std::ostream &log(Level level);

  /// @brief Set maximum logging level.
  static inline void setLoggingLevel(Level level);

 private:
  /// @brief Helper output stream used for disabled logging levels.
  class NullOStream : public std::ostream {
   public:
    NullOStream() : std::ostream(&mNullBuffer) {
    }
    virtual ~NullOStream() override { /* Automatic */
    }

   private:
    class NullBuffer : public std::streambuf {
     public:
      /// @brief Throw aray the input, returning success.
      int overflow(int c) {
        return c;
      }
    };  // class NullBuffer

    /// Internal buffer used for nulling the input.
    NullBuffer mNullBuffer{};
  };  // struct NullOStream

  /// @brief Convert level enumeration into unique index.
  static constexpr std::size_t levelToIdx(Level level);

  /// List of output streams used for logging.
  static inline std::ostream *sOutputStreams[LOG_LEVEL_COUNT + 1u]{};
  /// Null output stream used for disabled logging outputs.
  static inline NullOStream sNullOStream{};

 protected:
};  // class Logger

/// @brief Helper object used for indirect access to the loggin streams.
class LoggerAccess {
 public:
  /// @brief Type of manipulator used on the internal streams.
  using ManipT = std::ostream &(*)(std::ostream &);

  /// @brief Initialize the logger access for given log level.
  constexpr explicit LoggerAccess(Logger::Level level) : mLevel{level} {
    if (!sDefaultLoggingInitialized) {
      Logger::setLoggingLevel(Logger::DEFAULT_LOGGING_LEVEL);
      sDefaultLoggingInitialized = true;
    }
  }

  /// @brief Route stream manipulators.
  inline LoggerAccess &operator<<(ManipT manipulator);

  /// @brief Stream output routing.
  template <typename T>
  LoggerAccess &operator<<(const T &other);

  // @brief Get the corresponding output stream.
  inline std::ostream &ostream() const;

  // @brief Automatic conversion to output stream.
  inline operator std::ostream &() const;

 private:
  /// Which logging stream are we accessing?
  Logger::Level mLevel{Logger::Level::Debug};

  /// Flag used for default initializing the logging.
  static inline bool sDefaultLoggingInitialized{false};

 protected:
};  // struct LoggerAccess

/// @brief Debug messages logging stream.
static inline LoggerAccess Debug(Logger::Level::Debug);
/// @brief Info messages logging stream.
static inline LoggerAccess Info(Logger::Level::Info);
/// @brief Warning messages logging stream.
static inline LoggerAccess Warning(Logger::Level::Warning);
/// @brief Error messages logging stream.
static inline LoggerAccess Error(Logger::Level::Error);

/// @brief Run given code only when in debug mode.
#define DebugScopeStart if (treeutil::DebugWrapper::start()) {
/// @brief Run given code only when in debug mode.
#define DebugScopeEnd            \
  treeutil::DebugWrapper::end(); \
  }

/// @brief Class supporting debug operations.
class DebugWrapper {
 public:
  /**
   * @brief Start debugging scope.
   *
   * @return Returns true if debugging is enabled.
   * @usage Starting a debug only code block:
   *   if (treeutil::DebugWrapper::start())
   *   {
   *       // Runs only in debug mode...
   *       treeutil::DebugWrapper::end();
   *   }
   *   or:
   *   DebugScopeStart
   *       // Runs only in debug mode...
   *   DebugScopeEnd
   */
  static constexpr auto start() {
#ifdef NDEBUG
    return false;
#else   // !NDEBUG
    return true;
#endif  // !NDEBUG
  }

  /**
   * @brief End debugging scope.
   */
  static constexpr auto end() {
  }

 private:
 protected:
};  // class DebugScope

/// @brief Simple timer class using std::chrono::high_resolution_clock.
class Timer {
 public:
  /// @brief Clock used by this timer.
  using Clock = std::chrono::high_resolution_clock;
  /// @brief Type representing seconds elapsed.
  using SecondsT = double;

  /// @brief Initialize timer and start.
  inline Timer();

  /// @brief Reset the timer and return seconds elapsed since last reset.
  inline SecondsT reset();

  /// @brief Get seconds elapsed since last reset.
  inline SecondsT elapsed() const;

 private:
  /// Time at which this timer started counting.
  Clock::time_point mStart{};

 protected:
};  // class Timer

/// @brief Helper for printing progress bars in ASCII.
class ProgressBar {
 public:
  /// @brief Initialize the progress printer with given parameters.
  ProgressBar(const std::string &text, std::size_t width = 32u, bool rewrite = true);

  /// @brief Get progress-bar string for given percentage of completion in <0.0f, 1.0f>.
  std::string progress(float progress);

 private:
  /// Symbol used for empty parts of the progress bar.
  static constexpr auto EMPTY_SYMBOL{'='};
  /// Symbol used for full parts of the progress bar.
  static constexpr auto FULL_SYMBOL{'#'};
  /// Symbol used for the left border of the progress bar.
  static constexpr auto LEFT_BORDER_SYMBOL{'['};
  /// Symbol used for the right border of the progress bar.
  static constexpr auto RIGHT_BORDER_SYMBOL{']'};

  /// @brief Create empty progress bar string where each empty field has given symbol.
  static std::string emptyProgressString(std::size_t width, unsigned char symbol);

  /// @brief Create progress bar string for given parameters.
  static std::string progressString(std::size_t width, float progress, unsigned char emptySymbol,
                                    unsigned char fullSymbol, unsigned char leftBorderSymbol,
                                    unsigned char rightBorderSymbol);

  /// Text to prepend to the progress bar.
  std::string mText{};
  /// Width of the progress bar in a number of characters.
  std::size_t mWidth{};
  /// Have we already generated some progress strings?
  bool mProgressGenerated{};
  /// The last progress barr will be deleted before printing a new one if set to true.
  bool mRewrite{};

 protected:
};  // class ProgressBar

/// @brief Helper for printing progress bar reports.
template <typename T>
class ProgressPrinter {
 public:
  /// @brief Initialize the progress printer for given value count.
  ProgressPrinter(const ProgressBar &progressBar, const T &totalCount, std::size_t milestoneCount,
                  bool displayCount = true, bool displayTime = true);

  /// @brief Print progress if necessary and return whether printing occurred.
  template <typename StreamT>
  bool printProgress(StreamT &oStream, const T &currentCount, bool printNewLine = true);

 private:
  /// @brief Update the timer with current count.
  void updateTimer(const T &currentCount);

  /// @brief Prepare count string for display count functionality.
  std::string prepareCountString(const T &currentCount);
  /// @brief Prepare time string for display time functionality.
  std::string prepareTimeString(const T &currentCount);

  /// Progress bar used.
  ProgressBar mProgressBar{""};
  /// Total number of elements in the batch.
  T mTotalCount{};
  /// Number of milestones to print results at.
  std::size_t mMilestoneCount{};

  /// Last count provided to this object.
  T mLastCount{};
  /// Number of times we have already printed.
  std::size_t mPrintedCount{0u};

  /// Display current step count and total?
  bool mDisplayCount{};

  /// Display time information when printing progress?
  bool mDisplayTime{};
  /// Timer used for time display.
  Timer mTimer{};
  /// Time total so far.
  Timer::SecondsT mTimeTotal{};
  /// Estimated time per single step.
  Timer::SecondsT mTimePerStepEstimate{};
  /// Number of steps accumulated for step time estimate.
  std::size_t mTimeStepsAccumulated{};

 protected:
};  // class ProgressPrinter

#if 0

/// @brief Tuple of N elements of type T.
template <typename T, std::size_t N>
struct TupleN
{ template< typename...Args> using type = typename TupleN<T, N - 1u>::template type<T, Args...>; };

/// @brief Tuple of N elements of type T.
template <typename T>
struct TupleN<T, 0u>
{ template<typename...Args> using type = std::tuple<Args...>; };

/// @brief Tuple of N elements of type T.
template <typename T, std::size_t N>
using TupleOf = typename TupleN<T, N>::template type<>;

namespace impl
{

/// @brief Check range tuples for ==.
template <typename T, std::size_t N>
bool rangeTupleEqual(const TupleOf<T, N> &first, const TupleOf<T, N> &second);

/// @brief Check range tuples for <.
template <typename T, std::size_t N>
bool rangeTupleLess(const TupleOf<T, N> &first, const TupleOf<T, N> &second);

/// @brief Check range tuples for <=.
template <typename T, std::size_t N>
bool rangeTupleLessEqual(const TupleOf<T, N> &first, const TupleOf<T, N> &second);

/// @brief Check range tuples for >.
template <typename T, std::size_t N>
bool rangeTupleGreater(const TupleOf<T, N> &first, const TupleOf<T, N> &second);

/// @brief Check range tuples for >=.
template <typename T, std::size_t N>
bool rangeTupleGreaterEqual(const TupleOf<T, N> &first, const TupleOf<T, N> &second);

/// @brief Perform advance on given range tuple.
template <typename T, std::size_t N, std::size_t IDX = N>
TupleOf<T, IDX> rangeTupleAdvance(const TupleOf<T, N> &value,
    const TupleOf<T, N> &min, const TupleOf<T, N> &max,
    std::ptrdiff_t delta);

} // namespace impl

/// @brief Simple iterable range function.
template <typename T, std::size_t N>
class Range
{
public:
    /// @brief Value provided by this class.
    using value_type = TupleN<T, N>;

    /// @brief Iterator over a range.
    class RangeIterator
    {
    public:
        /// @brief Create default (end) range iterator.
        RangeIterator() = default;
        /// @brief Clean up and destroy.
        ~RangeIterator() = default;
        /// @brief Create iterator from range <min, max>.
        RangeIterator(const value_type &max, const value_type &min = { }) :
        { initialize(max, min); }

        // Copy and move constructors:
        RangeIterator(const RangeIterator &other) = default;
        RangeIterator &operator=(const RangeIterator &other) = default;
        RangeIterator(RangeIterator &&other) = default;
        RangeIterator &operator=(RangeIterator &&other) = default;

        // Comparison:
        bool operator==(const RangeIterator &other)
        { return (end && other.end) || impl::rangeTupleEqual(mCurrent, other.mCurrent); }
        bool operator!=(const RangeIterator &other)
        { return !(*this == other); }
        bool operator<(const RangeIterator &other)
        { return (!end && other.end) || impl::rangeTupleLess(mCurrent, other.mCurrent); }
        bool operator>(const RangeIterator &other)
        { return (!end && other.end) || impl::rangeTupleGreater(mCurrent, other.mCurrent); }
        bool operator<=(const RangeIterator &other)
        { return (*this == other) || (*this < other); }
        bool operator>=(const RangeIterator &other)
        { return (*this == other) || (*this > other); }

        // De-referencing:
        value_type operator*() const
        { return mCurrent; }
        value_type *operator->() const
        { return &mCurrent; }

        // Increment and decrement:
        RangeIterator &operator++()
        { advance(1u); return *this; }
        RangeIterator operator++(int)
        { auto copy{ *this }; operator++(); return copy; }
        RangeIterator &operator--()
        { advance(-1u); return *this; }
        RangeIterator operator--(int)
        { auto copy{ *this }; operator++(); return copy; }

        // Move by n:
        RangeIterator operator+(std::size_t n) const
        { auto copy{ *this }; copy.advance(n); }
        friend RangeIterator operator+(std::size_t n, const RangeIterator &rhs)
        { auto copy{ rhs }; copy.advance(n); }
        RangeIterator operator-(std::size_t n) const
        { auto copy{ *this }; copy.advance(-n); }
        friend RangeIterator operator-(std::size_t n, const RangeIterator &rhs)
        { auto copy{ rhs }; copy.advance(-n); }
        RangeIterator &operator+=(std::size_t n)
        { advance(n); return *this; }
        RangeIterator &operator-=(std::size_t n)
        { advance(-n); return *this; }

        // Indexing:
        const value_type &operator[](std::size_t n) const
        { auto copy{ *this }; copy.advance(n); return *copy; }

        /// @brief Advance this iterator by positive or negative amount. Returns wheter we are at the end.
        bool advance(std::ptrdiff_t delta);
    private:
        /// Current iterator value.
        value_type mCurrent{ };
        /// Minimal iterator value.
        value_type mMin{ };
        /// Maximmal iterator value.
        value_type mMax{ };
        /// Is this end iterator?
        bool mEnd{ true };
    protected:
    }; // class RangeIterator

    /// @brief Initialize empty range.
    Range() = default;
    /// @brief Clean up and destroy.
    ~Range() = default;

    /// @brief Initialize the range <min, max>.
    Range(const value_type &max, const value_type &min = { });
private:
protected:
}; // class Range

#endif

/// @brief Closure generator for calling instance method.
template <typename InstanceT, typename ReturnT, typename... ArgumentTs>
std::function<ReturnT(ArgumentTs...)> closure(InstanceT *instance, ReturnT (InstanceT::*method)(ArgumentTs...));

/**
 * @brief Validation fixture inspired by boost::hana.
 * @usage:
 *   is_valid<T>( [] (auto&& obj) -> decltype(obj.EXPR) { } )
 *   where <EXPR> is expression that is to be tested.
 */
template <typename T, typename F>
constexpr auto is_valid(F &&f) -> decltype(f(std::declval<T>()), true) {
  return true;
}

/// @brief Negative result for validity check.
template <typename>
constexpr bool is_valid(...) {
  return false;
}

/// @brief Helper for simple use of is_valid fixture.
#define IS_VALID(T, EXPR)                            \
  is_valid<T>([](auto &&obj) -> decltype(obj.EXPR) { \
    return {};                                       \
  })

/// @brief Remove const and reference from given type.
template <typename T>
using remove_const_reference = std::remove_const<typename std::remove_reference<T>::type>;

/// @brief Remove const and reference from given type.
template <typename T>
using remove_const_reference_t = typename remove_const_reference<T>::type;

/// @brief Get the other type if given type is void.
template <typename T, typename OtherT>
using other_than_void = std::conditional<std::is_void_v<T>, OtherT, T>;

/// @brief Get the other type if given type is void.
template <typename T, typename OtherT>
using other_than_void_t = typename other_than_void<T, OtherT>::type;

/// @brief Default case for specialization checking.
template <typename TestT, template <typename...> typename RefT>
struct is_specialization : std::false_type {};

/// @brief Positive case, where type is specialization of RefT.
template <template <typename...> class RefT, typename... ArgTs>
struct is_specialization<RefT<ArgTs...>, RefT> : std::true_type {};

/// @brief Check if TestT is a template specialization of RefT.
template <typename TestT, template <typename...> typename RefT>
static constexpr auto is_specialization_v{is_specialization<TestT, RefT>::value};

/// @brief Compare two strings for equality in case insensitive manner.
bool equalCaseInsensitive(const std::string &first, const std::string &second);

/// @brief Returns a path capped off by a slash - e.g. "/123/abc" -> "123/abc/". Useful for adding filenames to paths
std::string capPath(const std::string &filePath);

/// @brief Returns extension of given file or file path including the dot - e.g. "/123/abc.txt" -> ".txt".
std::string fileExtension(const std::string &filePath);

/// @brief Returns only path from given file path - e.g. "/123/abc.txt" -> "/123/".
std::string filePath(const std::string &filePath);

/// @brief Returns only base name from given file path - e.g. "/123/abc.txt" -> "abc".
std::string fileBaseName(const std::string &filePath);

/// @brief Get list of files with given extension. Extension must include dot.
std::vector<std::string> listFiles(const std::string &extension, const std::string &path = "", bool recursive = false,
                                   bool relative = false);

/// @brief Get list of files with given extensions. Extension must include dot.
std::vector<std::string> listFiles(const std::vector<std::string> &extensions, const std::string &path = "",
                                   bool recursive = false, bool relative = false);

/// @brief Does given file exist?
bool fileExists(const std::string &path);

/// @brief Delete file with given path, if it exists. Returns success.
bool deleteFile(const std::string &path);

/// @brief Read all of the given file into a string and return the result.
std::string readWholeFile(const std::string &path);

/// @brief Replace extension of the input path with given extension. Extension should include the ".".
std::string replaceExtension(const std::string &path, const std::string &extension);

/// @brief Convert given absolute path to relative path. Leave relative path empty to use current directory.
std::string relativePath(const std::string &path, const std::string &relativePath = "");

/// @brief Normalize given image data into RGB <0.0f, 1.0f> image.
template <typename T>
std::vector<Vector3D> convertImageNormalizedRGB(const std::vector<T> &data);

/// @brief Normalize given image data into RGB <0.0f, 1.0f> image.
template <typename ItT>
std::vector<Vector3D> convertImageNormalizedRGB(const ItT &begin, const ItT &end);

/// @brief Get maximum value such that value - delta == numeric_limits<VT>::min().
template <typename VT>
VT maximumNegativeDelta(const VT &val);

/// @brief Get maximum value such that value + delta == numeric_limits<VT>::max().
template <typename VT>
VT maximumPositiveDelta(const VT &val);

/// @brief Does given string contain only white spaces?
bool containsOnlyWhiteSpaces(const std::string &str);

/// @brief Calculate angle between two vectors in radians.
template <typename VT, typename VecT>
VT angleBetweenVectorsRad(const VecT &first, const VecT &second);

/// @brief Calculate angle between two normalized vectors in radians.
template <typename VT, typename VecT>
VT angleBetweenNormVectorsRad(const VecT &first, const VecT &second);

/// @brief Value of the number pi.
template <typename T>
static constexpr T PI{T(3.1415926535897932385L)};

/// @brief Convert radians to degrees.
template <typename VT>
VT radToDegrees(const VT &val);

/// @brief Convert degrees to radians.
template <typename VT>
VT degreesToRadians(const VT &val);

/// @brief Calculate smoothstep function - 3x^2 - 2x^3. Clamps automatically to <0, 1>.
template <typename VT>
VT smoothstep(const VT &val);

/// @brief Is given value in abs greater than epsilon?
template <typename VT>
bool aboveEpsilon(const VT &value, const VT &epsilon = std::numeric_limits<VT>::epsilon());

/// @brief Calculate volume of circular cone frustum.
template <typename VT>
VT circularConeFrustumVolume(const VT &h, const VT &r1, const VT &r2);

/**
 * @brief Break down duration into component durations. Inspired by: https://stackoverflow.com/a/42139394 .
 * @param duration Duration to be broken down.
 * @return Returns tuple containing all requested durations.
 * @usage breakDownDuration<std::chrono::seconds, std::chrono::milliseconds>(duration)
 */
template <typename... DurTs, typename DurT>
std::tuple<DurTs...> breakDownDuration(DurT duration);

/// @brief Create readable string from given duration - up to hours.
template <typename DurT>
std::string formatTime(const DurT &duration);

/// @brief Format an integer into its string hexadecimal representation.
template <typename T>
std::string formatIntHex(const T &val);

/// @brief Encode given binary data into string which can be stored in JSON format.
std::string encodeBinaryJSON(const std::vector<uint8_t> &data);

/// @brief Compress given data vector using HDF5 and return the resulting byte buffer.
std::vector<uint8_t> hdf5Compress(const std::vector<float> &data);
/// @brief Compress given data vector using HDF5 and return the resulting byte buffer.
std::vector<uint8_t> hdf5Compress(const std::vector<uint32_t> &data);
/// @brief Compress given data vector using HDF5 and return the resulting byte buffer.
std::vector<uint8_t> hdf5Compress(const std::vector<std::pair<float, float>> &data);
/// @brief Compress given data vector using HDF5 and return the resulting byte buffer.
std::vector<uint8_t> hdf5Compress(const std::vector<std::pair<uint32_t, float>> &data);

}  // namespace treeutil

// Template implementation begin.

namespace treeutil {

#if 0

namespace impl
{

template <typename T, std::size_t N>
bool rangeTupleEqual(const TupleOf<T, N> &first, const TupleOf<T, N> &second)
{ return first == second; }

template <typename T, std::size_t N>
bool rangeTupleLess(const TupleOf<T, N> &first, const TupleOf<T, N> &second)
{ return first < second; }

template <typename T, std::size_t N>
bool rangeTupleLessEqual(const TupleOf<T, N> &first, const TupleOf<T, N> &second)
{ return first <= second; }

template <typename T, std::size_t N>
bool rangeTupleGreater(const TupleOf<T, N> &first, const TupleOf<T, N> &second)
{ return first > second; }

template <typename T, std::size_t N>
bool rangeTupleGreaterEqual(const TupleOf<T, N> &first, const TupleOf<T, N> &second)
{ return first >= second; }

template <typename T, std::size_t N, std::size_t IDX>
TupleOf<T, IDX> rangeTupleAdvance(const TupleOf<T, N> &value,
    const TupleOf<T, N> &min, const TupleOf<T, N> &max,
    std::ptrdiff_t delta)
{
    //const auto intervalLength{ std::get<IDX>(max) - std::get<IDX>(min) };
    //std::get<IDX>(value) - std::get<IDX>(min)

    /*
    if (delta < 0)
    {
        if (std::get<IDX>(value) + delta) < std::get<IDX>(min)
    }
     */

    return std::tuple_cat(
        rangeTupleAdvance<T, N, IDX - 1u>(value, min, max,
            delta / (std::get<IDX>(max) - std::get<IDX>(min))),
        std::tuple<T>{ 1u }
    );
}

/*
template <typename T, std::size_t N>
TupleOf<T, 1u> rangeTupleAdvance<T, N, 1u>(const TupleOf<T, N> &value,
    const TupleOf<T, N> &min, const TupleOf<T, N> &max,
    std::ptrdiff_t delta)
{
    return std::tuple{ 0u };
}
 */

}

template <typename T, std::size_t N>
bool Range<T, N>::RangeIterator::advance(std::ptrdiff_t delta)
{
    if (mEnd)
    { return true; }

    if (delta > 0)
    {
        mCurrent = impl::rangeTupleAdvancePositive<N>(mCurrent, mMax, delta);
        if (impl::rangeTupleGreaterEqual(mCurrent, mMax))
        { mEnd = true; }
    }
    else if (delta < 0)
    {
        mCurrent = impl::rangeTupleAdvanceNegative<N>(mCurrent, mMin, delta);
        if (impl::rangeTupleLessEqual(mCurrent, mMin))
        { mEnd = true; }
    }

    return mEnd;
}

#endif

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::CopyableSmartPtr(T *pointer) : ptr{pointer} {
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::CopyableSmartPtr(const PtrT<T> &pointer) : ptr{pointer} {
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::CopyableSmartPtr(const CopyableSmartPtr &other) {
  if constexpr (IS_VALID(T, duplicate())) {
    if (other.ptr) {
      ptr = PtrT<T>{other.ptr->duplicate()};
    }
  } else if constexpr (IS_VALID(T, clone())) {
    if (other.ptr) {
      ptr = PtrT<T>{other.ptr->clone()};
    }
  } else {
    if (other.ptr) {
      ptr = PtrT<T>{new T{*other.ptr}};
    }
  }
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT> &CopyableSmartPtr<T, PtrT>::operator=(CopyableSmartPtr other) {
  swap(other);
  return *this;
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::CopyableSmartPtr(CopyableSmartPtr &&other) {
  swap(other);
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT> &CopyableSmartPtr<T, PtrT>::operator=(CopyableSmartPtr &&other) {
  swap(other);
  return *this;
}

template <typename T, template <typename> typename PtrT>
void CopyableSmartPtr<T, PtrT>::swap(CopyableSmartPtr &other) {
  swap(*this, other);
}

template <typename T, template <typename> typename PtrT>
void CopyableSmartPtr<T, PtrT>::swap(CopyableSmartPtr &first, CopyableSmartPtr &second) {
  using std::swap;
  swap(first.ptr, second.ptr);
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::operator PtrT<T>() {
  return ptr;
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::operator PtrT<T>() const {
  return ptr;
}

template <typename T, template <typename> typename PtrT>
T &CopyableSmartPtr<T, PtrT>::operator*() {
  return ptr.operator*();
}

template <typename T, template <typename> typename PtrT>
const T &CopyableSmartPtr<T, PtrT>::operator*() const {
  return ptr.operator*();
}

template <typename T, template <typename> typename PtrT>
T *CopyableSmartPtr<T, PtrT>::operator->() {
  return ptr.operator->();
}

template <typename T, template <typename> typename PtrT>
const T *CopyableSmartPtr<T, PtrT>::operator->() const {
  return ptr.operator->();
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT>::operator bool() const {
  return ptr.operator bool();
}

template <typename T, template <typename> typename PtrT>
CopyableSmartPtr<T, PtrT> &CopyableSmartPtr<T, PtrT>::operator=(const PtrT<T> &other) {
  ptr = other;
  return *this;
}

template <typename InputItT, typename LambdaT>
LambdaIterator<InputItT, LambdaT>::LambdaIterator(const InputItT &iterator, const LambdaT &lambda)
    : InputItT{iterator}, mLambda{lambda} {
}
template <typename InputItT, typename LambdaT>
LambdaIterator<InputItT, LambdaT>::~LambdaIterator() { /* Automatic */
}

template <typename InputItT, typename LambdaT>
auto LambdaIterator<InputItT, LambdaT>::operator*() {
  return mLambda(InputItT::operator*());
}
template <typename InputItT, typename LambdaT>
auto LambdaIterator<InputItT, LambdaT>::operator*() const {
  return mLambda(InputItT::operator*());
}
template <typename InputItT, typename LambdaT>
auto LambdaIterator<InputItT, LambdaT>::operator->() {
  return mLambda(InputItT::operator->());
}
template <typename InputItT, typename LambdaT>
auto LambdaIterator<InputItT, LambdaT>::operator->() const {
  return mLambda(InputItT::operator->());
}
template <typename InputItT, typename LambdaT>
auto LambdaIterator<InputItT, LambdaT>::operator[](std::size_t n) {
  return mLambda(InputItT::operator[](n));
}
template <typename InputItT, typename LambdaT>
auto LambdaIterator<InputItT, LambdaT>::operator[](std::size_t n) const {
  return mLambda(InputItT::operator[](n));
}

template <typename T>
T uniformZeroToOne() {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::uniform_real_distribution<T> dis(0.0, 1.0);

  return dis(gen);
}

template <typename T>
T sgn(const T &val) {
  return (T(0) < val) - (val < T(0));
}

template <typename ArrT>
auto minMax(const ArrT &arr) {
  using ValueT = typename remove_const_reference<decltype(*arr.begin())>::type;

  ValueT minVal{std::numeric_limits<ValueT>::max()};
  ValueT maxVal{std::numeric_limits<ValueT>::lowest()};

  for (const auto &val : arr) {
    minVal = std::min<ValueT>(minVal, val);
    maxVal = std::max<ValueT>(maxVal, val);
  }

  return std::pair<ValueT, ValueT>{minVal, maxVal};
}

template <typename ArrT>
auto argMin(const ArrT &arr) {
  using ValueT = typename remove_const_reference<decltype(*arr.begin())>::type;

  auto argMin{std::numeric_limits<std::size_t>::max()};
  ValueT minVal{std::numeric_limits<ValueT>::max()};

  std::size_t indexCounter{0u};
  for (const auto &val : arr) {
    if (val <= minVal) {
      minVal = val;
      argMin = indexCounter;
    }
    indexCounter++;
  }

  return std::tuple<ValueT, std::size_t>{minVal, argMin};
}

template <typename ArrT>
auto argMax(const ArrT &arr) {
  using ValueT = typename remove_const_reference<decltype(*arr.begin())>::type;

  auto argMax{std::numeric_limits<std::size_t>::max()};
  ValueT maxVal{std::numeric_limits<ValueT>::lowest()};

  std::size_t indexCounter{0u};
  for (const auto &val : arr) {
    if (val >= maxVal) {
      maxVal = val;
      argMax = indexCounter;
    }
    indexCounter++;
  }

  return std::tuple<ValueT, std::size_t>{maxVal, argMax};
}

template <typename ArrT>
auto argMinMax(const ArrT &arr) {
  using ValueT = typename remove_const_reference<decltype(*arr.begin())>::type;

  auto argMin{std::numeric_limits<std::size_t>::max()};
  auto minVal{std::numeric_limits<ValueT>::max()};
  auto argMax{std::numeric_limits<std::size_t>::max()};
  auto maxVal{std::numeric_limits<ValueT>::lowest()};

  std::size_t indexCounter{0u};
  for (const auto &val : arr) {
    if (val <= minVal) {
      minVal = val;
      argMin = indexCounter;
    }
    if (val >= maxVal) {
      maxVal = val;
      argMax = indexCounter;
    }
    indexCounter++;
  }

  return std::tuple<ValueT, std::size_t, ValueT, std::size_t>{minVal, argMin, maxVal, argMax};
}

inline std::ostream &Logger::log(Level level) {
  return *sOutputStreams[levelToIdx(level)];
}

inline LoggerAccess &LoggerAccess::operator<<(LoggerAccess::ManipT manipulator) {
  manipulator(Logger::log(mLevel));
  return *this;
}

template <typename T>
LoggerAccess &LoggerAccess::operator<<(const T &other) {
  Logger::log(mLevel) << other;
  return *this;
}

inline std::ostream &LoggerAccess::ostream() const {
  return Logger::log(mLevel);
}

inline LoggerAccess::operator std::ostream &() const {
  return ostream();
}

inline void Logger::setLoggingLevel(Level level) {
  if (levelToIdx(level) <= levelToIdx(Level::Debug)) {
    sOutputStreams[levelToIdx(Level::Debug)] = &std::cout;
  } else {
    sOutputStreams[levelToIdx(Level::Debug)] = &sNullOStream;
  }

  if (levelToIdx(level) <= levelToIdx(Level::Info)) {
    sOutputStreams[levelToIdx(Level::Info)] = &std::cout;
  } else {
    sOutputStreams[levelToIdx(Level::Info)] = &sNullOStream;
  }

  if (levelToIdx(level) <= levelToIdx(Level::Warning)) {
    sOutputStreams[levelToIdx(Level::Warning)] = &std::clog;
  } else {
    sOutputStreams[levelToIdx(Level::Warning)] = &sNullOStream;
  }

  if (levelToIdx(level) <= levelToIdx(Level::Error)) {
    sOutputStreams[levelToIdx(Level::Error)] = &std::cerr;
  } else {
    sOutputStreams[levelToIdx(Level::Error)] = &sNullOStream;
  }
}

constexpr std::size_t Logger::levelToIdx(Level level) {
  switch (level) {
    default:
    case Level::Debug: {
      return 0u;
    }
    case Level::Info: {
      return 1u;
    }
    case Level::Warning: {
      return 2u;
    }
    case Level::Error: {
      return 3u;
    }
  }
}

Timer::Timer() {
  reset();
}

inline Timer::SecondsT Timer::reset() {
  const auto result{elapsed()};
  mStart = Clock::now();
  return result;
}

inline Timer::SecondsT Timer::elapsed() const {
  return std::chrono::duration_cast<std::chrono::duration<SecondsT>>(Clock::now() - mStart).count();
}

template <typename T>
ProgressPrinter<T>::ProgressPrinter(const ProgressBar &progressBar, const T &totalCount, std::size_t milestoneCount,
                                    bool displayCount, bool displayTime)
    : mProgressBar{progressBar},
      mTotalCount{totalCount},
      mMilestoneCount{milestoneCount},
      mLastCount{},
      mPrintedCount{0u},
      mDisplayCount{displayCount},
      mDisplayTime{displayTime},
      mTimer{},
      mTimeTotal{},
      mTimePerStepEstimate{},
      mTimeStepsAccumulated{} {
  mTimer.reset();
}

template <typename T>
template <typename StreamT>
bool ProgressPrinter<T>::printProgress(StreamT &oStream, const T &currentCount, bool printNewLine) {
  if (mDisplayTime) {
    updateTimer(currentCount);
  }

  const auto shouldHavePrinted{currentCount / (mTotalCount / mMilestoneCount)};
  if (shouldHavePrinted > mPrintedCount) {
    oStream << mProgressBar.progress(static_cast<float>(currentCount) / mTotalCount);
    if (mDisplayCount) {
      oStream << " " << prepareCountString(currentCount);
    }
    if (mDisplayTime) {
      oStream << " " << prepareTimeString(currentCount);
    }
    if (printNewLine) {
      oStream << std::endl;
    }
    mPrintedCount = shouldHavePrinted;
    return true;
  }

  mLastCount = currentCount;
  if (mDisplayTime) {
    mTimer.reset();
  }

  return false;
}

template <typename T>
void ProgressPrinter<T>::updateTimer(const T &currentCount) {
  const auto deltaSeconds{mTimer.elapsed()};
  const auto newSteps{currentCount - mLastCount};
  const auto secondsPerStep{newSteps > T{} ? deltaSeconds / newSteps : 0};

  // Update per-step estimate.
  if (mTimeStepsAccumulated == 0u) {  // No previous data -> use only current estimate.
    mTimePerStepEstimate = secondsPerStep;
  } else {  // We have previous samples -> filter them together with new samples.
    mTimePerStepEstimate =
        (mTimePerStepEstimate * mTimeStepsAccumulated + secondsPerStep * newSteps) / (mTimeStepsAccumulated + newSteps);
  }
  mTimeStepsAccumulated += newSteps;

  // Update time total.
  mTimeTotal += deltaSeconds;
}

template <typename T>
std::string ProgressPrinter<T>::prepareCountString(const T &currentCount) {
  // Generate the count string.
  std::stringstream countString{};
  const auto totalCountString{std::to_string(mTotalCount)};
  countString << "[" << std::setfill('0') << std::setw(totalCountString.size()) << currentCount << " / " << mTotalCount
              << "]";

  return countString.str();
}

template <typename T>
std::string ProgressPrinter<T>::prepareTimeString(const T &currentCount) {
  static constexpr auto S_TO_MS{1000.0};
  const auto timeTotalMs{static_cast<std::size_t>(mTimeTotal * S_TO_MS)};
  const auto timeTotalCompleteMs{
      static_cast<std::size_t>(timeTotalMs + ((mTotalCount - currentCount) * mTimePerStepEstimate) * S_TO_MS)};
  const auto timePerStepMs{static_cast<std::size_t>(mTimePerStepEstimate * S_TO_MS)};

  // Generate the time string.
  std::stringstream timeString{};
  timeString << "{" << formatTime(std::chrono::milliseconds{timeTotalMs}) << " / ~"
             << formatTime(std::chrono::milliseconds{timeTotalCompleteMs}) << "} ("
             << formatTime(std::chrono::milliseconds{timePerStepMs}) << " / step)";

  return timeString.str();
}

template <typename T>
std::vector<Vector3D> convertImageNormalizedRGB(const std::vector<T> &data) {
  return convertImageNormalizedRGB(data.begin(), data.end());
}

template <typename ItT>
std::vector<Vector3D> convertImageNormalizedRGB(const ItT &begin, const ItT &end) {
  using T = typename std::remove_const<typename std::remove_reference<decltype(*begin)>::type>::type;
  auto min{std::numeric_limits<T>::max()};
  auto max{std::numeric_limits<T>::min()};
  for (auto it = begin; it != end; ++it) {
    min = std::min(min, *it);
    max = std::max(max, *it);
  }

  std::vector<Vector3D> converted{};
  converted.resize(static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, std::distance(begin, end))));
  std::size_t idx{0u};
  for (auto it = begin; it != end; ++it, ++idx) {
    const auto quantized{Vector3D{(*it - min)} / (max - min)};
    converted[idx] = quantized;
  }

  return converted;
}

/*
template <>
inline std::vector<Vector3D> convertImageNormalizedRGB<Vector3D>(const std::vector<Vector3D> &data)
{
    auto min{ Vector3D::maxVector() };
    auto max{ Vector3D::minVector() };
    for (const auto &value : data)
    { min = Vector3D::elementMin(min, value); max = Vector3D::elementMax(max, value); }

    std::vector<Vector3D> converted{ };
    converted.resize(data.size());
    for (std::size_t iii = 0u; iii < data.size(); ++iii)
    { const auto quantized{ ((data[iii] - min) / (max - min)) }; converted[iii] = quantized; }

    return converted;
}
 */

template <typename VT>
VT maximumNegativeDelta(const VT &val) {
  return std::max<VT>(val - std::numeric_limits<VT>::min(), VT{});
}

template <typename VT>
VT maximumPositiveDelta(const VT &val) {
  return std::max<VT>(std::numeric_limits<VT>::max() - val, VT{});
}

template <typename VT, typename VecT>
VT angleBetweenVectorsRad(const VecT &first, const VecT &second) {
  return static_cast<VT>(std::acos(std::clamp<VT>(first.normalized().dot(second.normalized()), -1, 1)));
}

template <typename VT, typename VecT>
VT angleBetweenNormVectorsRad(const VecT &first, const VecT &second) {
  return static_cast<VT>(std::acos(std::clamp<VT>(first.dot(second), -1, 1)));
}

template <typename VT>
VT radToDegrees(const VT &val) {
  return (VT(180.0L) / ::treeutil::PI<VT>)*val;
}

template <typename VT>
VT degreesToRadians(const VT &val) {
  return (::treeutil::PI<VT> / VT(180.0L)) * val;
}

template <typename VT>
VT smoothstep(const VT &val) {
  const auto clamped{std::clamp(val, VT(0), VT(1))};
  const auto x2{clamped * clamped};
  const auto x3{x2 * clamped};
  return VT(3) * x2 - VT(2) * x3;
}

template <typename VT>
bool aboveEpsilon(const VT &value, const VT &epsilon) {
  if constexpr (std::numeric_limits<VT>::is_signed) {
    return std::abs(value) > epsilon;
  } else {
    return value > epsilon;
  }
}

template <typename VT>
VT circularConeFrustumVolume(const VT &h, const VT &r1, const VT &r2) {
  return (treeutil::PI<VT> * h) / 3.0f * (r1 * r1 + r1 * r2 + r2 * r2);
}

template <typename... DurTs, typename DurT>
std::tuple<DurTs...> breakDownDuration(DurT duration) {
  std::tuple<DurTs...> result{};
  using discard = int[];

  // Constexpr for_each over DurTs:
  (void)discard{0, (void(((std::get<DurTs>(result) = std::chrono::duration_cast<DurTs>(duration)),
                          (duration -= std::chrono::duration_cast<DurT>(std::get<DurTs>(result))))),
                    0)...};

  return result;
}

template <typename DurT>
std::string formatTime(const DurT &duration) {
  const auto [hours, minutes, seconds, milliseconds]{
      breakDownDuration<std::chrono::hours, std::chrono::minutes, std::chrono::seconds, std::chrono::milliseconds>(
          duration)};

  std::stringstream ss{};
  ss << std::setfill('0') << std::setw(2) << hours.count() << ":" << std::setfill('0') << std::setw(2)
     << minutes.count() << ":" << std::setfill('0') << std::setw(2) << seconds.count() << "." << std::setfill('0')
     << std::setw(3) << milliseconds.count();
  return ss.str();
}

template <typename T>
std::string formatIntHex(const T &val) {
  std::stringstream ss{};
  ss << "0x" << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex << val;
  return ss.str();
}

}  // namespace treeutil

namespace std {

static inline treeutil::LoggerAccess &endl(treeutil::LoggerAccess &stream) {
  stream.ostream() << std::endl;
  return stream;
}

template <typename T1, typename T2>
static inline auto operator+(const std::pair<T1, T2> &first, const std::pair<T1, T2> &second) {
  return std::make_pair(first.first + second.first, first.second + second.second);
}

}  // namespace std

// Template implementation end.

#endif  // TREEIO_UTILS_H
