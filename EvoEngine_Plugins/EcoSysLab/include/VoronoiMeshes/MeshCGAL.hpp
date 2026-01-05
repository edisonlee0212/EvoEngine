#ifdef USE_CGAL
#  include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#  include <CGAL/Surface_mesh.h>

namespace kinDS {
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using MeshCGAL_internal = CGAL::Surface_mesh<Kernel::Point_3>;
/**
 * A wrapper class for CGAL meshes that attaches a property map and implements a deep copy to preserve the property map.
 */
template <class PropertyType>
class MeshCGAL {
 public:
  using Point = Kernel::Point_3;
  using PropertyMapType = MeshCGAL_internal::Property_map<CGAL::SM_Face_index, PropertyType>;

  using FaceDescriptor = MeshCGAL_internal::Face_index;
  using vertex_descriptor = MeshCGAL_internal::Vertex_index;

  // ---- STORED DATA ----
  MeshCGAL_internal mesh;
  PropertyMapType fidx;

  std::string property_name;

 private:
  PropertyType default_value;

  // ---- CONSTRUCTORS ----
 public:
  MeshCGAL(std::string property_name = "f:my_property", PropertyType default_value = PropertyType())
      : property_name(property_name), default_value(default_value) {
    // Create the property maps
    auto pair = mesh.add_property_map<FaceDescriptor, PropertyType>(property_name, default_value);
    fidx = pair.first;
  }

  // ---- COPY CONSTRUCTOR ----
  MeshCGAL(const MeshCGAL& other) {
    deep_copy_from(other);
  }

  // ---- COPY ASSIGNMENT ----
  MeshCGAL& operator=(const MeshCGAL& other) {
    if (this != &other)
      deep_copy_from(other);
    return *this;
  }

  // ---- MOVE (fine with default) ----
  MeshCGAL(MeshCGAL&&) = default;
  MeshCGAL& operator=(MeshCGAL&&) = default;

 private:
  // ---- INTERNAL DEEP COPY FUNCTION ----
  void deep_copy_from(const MeshCGAL& other) {
    // 1. Copy the mesh
    property_name = other.property_name;
    default_value = other.default_value;
    mesh = other.mesh;

    // 2. Recreate + copy custom property maps
    {
      auto pm = mesh.add_property_map<FaceDescriptor, PropertyType>(property_name, default_value);
      fidx = pm.first;

      for (FaceDescriptor f : mesh.faces()) {
        // Copy value from 'other'
        FaceDescriptor f_other(f);  // because the indexing is identical
        fidx[f] = other.fidx[f_other];
      }
    }
  }
};
}  // namespace kinDS
#endif