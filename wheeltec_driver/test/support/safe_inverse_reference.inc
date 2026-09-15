// Test-only replacement for the vendor inverse-square-root memory access.
// Keep volatile arithmetic; use a fixed-width object and memcpy for the bit cast.
float InvSqrt(float number)
{
  uint32_t i;
  volatile float x, y;
  volatile const float f = 1.5F;
  x = number * 0.5F;
  y = number;
  float bits_source = y;
  std::memcpy(&i, &bits_source, sizeof(i));
  i = 0x5f375a86U - (i >> 1);
  std::memcpy(&bits_source, &i, sizeof(bits_source));
  y = bits_source;
  y = y * (f - (x * y * y));
  return y;
}
