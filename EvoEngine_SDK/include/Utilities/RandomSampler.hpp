
#pragma once

namespace evo_engine {

/**
 * @class RandomSampler
 * @brief A class to generate random samples for rendering or simulation.
 */
class RandomSampler {
 public:
  /**
   * @brief Default constructor for RandomSampler.
   */
  RandomSampler() = default;

  /**
   * @brief Sets the seed for the random number generator.
   *
   * @param start_state The starting state for the RNG.
   * @param init_seq The initial sequence number (default is 1).
   */
  void SetSeed(const uint64_t start_state, const uint64_t init_seq = 1) {
    state_ = 0U;
    inc_ = init_seq << 1 | 1u;
    NextUint();
    state_ += start_state;
    NextUint();
  }

  /**
   * @brief Configures the RNG based on a pixel index and a sample index.
   *
   * @param pixel_index The index of the pixel.
   * @param sample_index The sample index.
   */
  void SetPixelSample(const int pixel_index, const uint64_t sample_index) {
    uint64_t x = pixel_index & 0x0000ffff;  // x = ---- ---- ---- ---- fedc ba98 7654 3210
    x = (x | x << 8) & 0x00FF00FF;          // x = ---- ---- fedc ba98 ---- ---- 7654 3210
    x = (x | x << 4) & 0x0F0F0F0F;          // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
    x = (x | x << 2) & 0x33333333;          // x = --fe --dc --ba --98 --76 --54 --32 --10
    x = (x | x << 1) & 0x55555555;          // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
    const uint64_t s0 = x;
    const uint64_t s1 = sample_index;
    SetSeed(s0, s1);
  }

  /**
   * @brief Generates a random floating-point number in the range [0, 1).
   *
   * @return A random float.
   */
  float Get1D() {
    return NextFloat();
  }

  /**
   * @brief Generates a random 2D point with x and y in the range [0, 1).
   *
   * @return A glm::vec2 representing the random point.
   */
  glm::vec2 Get2D() {
    return {Get1D(), Get1D()};
  }

  /**
   * @brief Advances the RNG by a specified number of steps.
   *
   * @param delta The number of steps to advance (default is 1 << 32).
   */
  void Advance(int64_t delta = 1ll < 32) {
    uint64_t cur_multiplier = 0x5851f42d4c957f2dULL, cur_plus = inc_, acc_multiplier = 1u, acc_plus = 0u;

    while (delta > 0) {
      if (delta & 1) {
        acc_multiplier *= cur_multiplier;
        acc_plus = acc_plus * cur_multiplier + cur_plus;
      }
      cur_plus = (cur_multiplier + 1) * cur_plus;
      cur_multiplier *= cur_multiplier;
      delta /= 2;
    }
    state_ = acc_multiplier * state_ + acc_plus;
  }

 private:
  /**
   * @brief Generates the next unsigned integer from the RNG.
   *
   * @return A 32-bit unsigned random integer.
   */
  uint32_t NextUint() {
    const uint64_t prev_state = state_;
    state_ = prev_state * 0x5851f42d4c957f2dULL + inc_;
    const uint32_t xor_shifted = static_cast<uint32_t>((prev_state >> 18u ^ prev_state) >> 27u);
    const uint32_t rot = static_cast<uint32_t>(prev_state >> 59u);
    return xor_shifted >> rot | xor_shifted << (~rot + 1u & 31);
  }

  /**
   * @brief Generates the next random double in the range [0, 1).
   *
   * @return A random double.
   */
  double NextDouble() {
    union {
      uint64_t u;
      double d;
    } x;
    x.u = static_cast<uint64_t>(NextUint()) << 20 | 0x3ff0000000000000ULL;
    return x.d - 1.0;
  }

  /**
   * @brief Generates the next random float in the range [0, 1).
   *
   * @return A random float.
   */
  float NextFloat() {
    union {
      uint32_t u;
      float f;
    } x;
    x.u = NextUint() >> 9 | 0x3f800000u;
    return x.f - 1.0f;
  }

  uint64_t state_;  ///< The current state of the RNG. All values are possible.
  uint64_t inc_;    ///< Controls which RNG sequence (stream) is selected. Must always be odd.
};

}  // namespace evo_engine
