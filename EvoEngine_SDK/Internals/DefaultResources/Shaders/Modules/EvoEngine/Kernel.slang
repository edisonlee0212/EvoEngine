// @evoengine-dialect native
public static const int EE_MAX_KERNEL_AMOUNT = 64;

public struct EeKernelBlock {
  public float4 uniform_kernel[EE_MAX_KERNEL_AMOUNT];
  public float4 gauss_kernel[EE_MAX_KERNEL_AMOUNT];
};

[[vk::binding(5, 0)]]
public ConstantBuffer<EeKernelBlock, Std140DataLayout> EE_KERNEL_BLOCK;
