class DynamicStrandUtils {
 public:
  static float PointPlaneDistance(const glm::vec3& target_point, const glm::vec3& target_a, const glm::vec3& target_b,
                           const glm::vec3& target_c);
  static std::pair<int, int> CompareIndices(const int a[4], const int b[4]);
  static bool IsValid(const int target_indices[4], int size);
};