# PhysXPhysics Service

[Back to Service index](../README.md)

PhysXPhysics contains an unfinished or currently disabled PhysX integration. The code suggests a physics layer with rigid bodies, colliders, joints, and physics materials, but the Service does not currently participate in the normal build.

## Build Status

- Present in `EvoEngine_Services/PhysXPhysics`.
- Disabled by default through `ServiceInfo.cmake`.
- Its own `CMakeLists.txt` begins with `return()`, so it exits immediately even if added manually.
- Intended target name is `PhysXPhysicsService`.
- Intended compile definition is `PHYSX_PHYSICS_SERVICE`.

## Main Responsibilities

The existing headers indicate support for:

- `PhysicsLayer`
- `RigidBody`
- `Collider`
- `Joint`
- `PhysicsMaterial`

## SDK Integration

The intended design appears to be a Service layer plus private components/assets for physics behavior. Because the CMake file returns immediately, this should be treated as inactive code until the build configuration and dependencies are restored.

## Future Work Notes

Before using this Service, decide whether PhysX support is still desired. Enabling it will require removing the early `return()`, verifying the third-party PhysX paths, registering the Service from the parent CMake, checking DLL/SO copy behavior, and then validating component registration and runtime lifecycle.
