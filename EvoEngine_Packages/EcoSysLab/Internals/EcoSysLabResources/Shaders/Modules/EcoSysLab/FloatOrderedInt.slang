// This provides a function to convert a float to an order-preserving int and back. This is needed when using
// atomicMin() or atomicMax() on floats, which is only available for int.

// Convert float to order-preserving int
public int floatToOrderedInt(float f) {
  int i = asint(f);
  return (i >= 0) ? i : (0x80000000 - i);
}
// Convert back
public float orderedIntToFloat(int i) {
  return asfloat((i >= 0x80000000) ? (0x80000000 - i) : i);
}
