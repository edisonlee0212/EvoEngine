# Stochastic screen-space reflections

[Back to rendering overview](rendering.md)

SSR traces one GGX visible-normal reflection sample per pixel per rendered frame, using the same
visible-normal sampler, Schlick Fresnel and separable Smith masking functions as PT. The random sequence
uses a pixel-scrambled R2 sequence indexed by the application frame count, not the frame-in-flight slot or
PT accumulation count. Integer arithmetic preserves the sequence at large frame indices. Smooth materials
use the exact mirror direction. Deferred shading publishes its resolved reflection fallback into the existing
SSR swap texture: reflection probes take priority, followed by the active DDGI/SDFGI/HDDAGI provider,
then environment lighting where GI has no valid coverage. SSR consumes the same fallback as the shaded
scene, so unavailable GI does not replace environment lighting with black. Valid dark GI remains dark.
Reflection RGB contains the signed difference between screen and fallback radiance, multiplied by
BRDF/PDF weight, hit confidence and specular visibility; alpha contains coverage. Equal screen and
fallback radiance produce zero correction. Spatial and temporal reconstruction average both channels,
including missed rays. Composition adds this correction to existing lighting without applying the BRDF
or visibility twice. The scratch texture is reused; no separate GI query pass or image allocation is needed.
Geometry/material rejection discards invalid history. Neighborhood clipping ramps in over the first pixel of
reprojection motion; stationary surfaces retain valid history across changing stochastic samples. Lighting and
reflected-object changes still respond through temporal averaging. Stationary stochastic surfaces use
95-99% history weight (roughness dependent); motion retains the existing 85-97% weights. This trades
slower response to lighting changes for lower noise; mirrors retain their existing history weights. Spatial reconstruction checks depth,
normals, roughness and specular F0. With spatial resolve and temporal stabilization enabled, two additional
5x5 passes filter the temporal result at increasing radii. Their footprint scales with roughness, and local RGB
variance controls the blend strength. The passes preserve signed reflection corrections and coverage, reuse
existing scratch textures, and do not feed spatially blurred results back into temporal history.

In the PostProcessingStack asset inspector, expand **SSR**. SSR fades to existing probe/environment
specular between **Roughness fade start** (default 0.4) and **Roughness fade end** (default 0.8).
**Roughness filter radius (1080p pixels)** (default 12, range 0-32) scales the edge-aware spatial filter
by squared material roughness and viewport height in the initial pass, and by roughness with increasing scale
in the post-temporal passes; enable **Edge-aware spatial resolve** to use it.
Enable **Temporal stabilization** to smooth the changing samples. Disabling reconstruction exposes
single-frame sampling noise. The controls are serialized; older assets use the new defaults.
Screen-space visibility, spatial reuse and probe substitution remain approximations.
