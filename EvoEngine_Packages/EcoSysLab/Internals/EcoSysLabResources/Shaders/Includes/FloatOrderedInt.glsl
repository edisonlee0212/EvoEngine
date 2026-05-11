// This provides a function to convert a float to an order-preserving int and back. This is needed when using
// atomicMin() or atomicMax() on floats, which is only available for int.

// Convert float to order-preserving int
int floatToOrderedInt(float f) {
  int i = floatBitsToInt(f);
  return (i >= 0) ? i : (0x80000000 - i);
}

// Convert back
float orderedIntToFloat(int i) {
  return intBitsToFloat((i >= 0x80000000) ? (0x80000000 - i) : i);
}