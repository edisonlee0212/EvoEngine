
// naive comparison
bool is_larger(in const HashedGridElement left, in const HashedGridElement right) {
  return left.cell_id > right.cell_id;
}

// Pick comparison funtion: for colors we might want to compare perceptual brightness
// instead of a naive straight integer value comparison.
#define COMPARE is_larger