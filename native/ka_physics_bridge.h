#ifndef KA_PHYSICS_BRIDGE_H
#define KA_PHYSICS_BRIDGE_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(KA_PHYSICS_BRIDGE_EXPORTS)
#    define KA_API __declspec(dllexport)
#  else
#    define KA_API __declspec(dllimport)
#  endif
#else
#  define KA_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum { KA_PHYSICS_ABI_VERSION = 1 };

typedef struct KAPhysicsVec3 {
    float x;
    float y;
    float z;
} KAPhysicsVec3;

typedef struct KAPhysicsQuat {
    float w;
    float x;
    float y;
    float z;
} KAPhysicsQuat;

typedef struct KAPhysicsTransform {
    KAPhysicsVec3 position;
    KAPhysicsQuat rotation;
} KAPhysicsTransform;

KA_API int ka_physics_abi_version(void);
KA_API const char *ka_physics_backend_name(void);
KA_API const char *ka_physics_last_error(void);

/* Reserved ABI for the first native milestone. These functions are not called
 * by add-on version 0.1.0. Keeping the declarations stable allows Jolt and
 * PhysX implementations to share the same Blender-side ctypes adapter. */
KA_API void *ka_world_create(KAPhysicsVec3 gravity, uint32_t max_bodies);
KA_API void ka_world_destroy(void *world);
KA_API int ka_world_step(void *world, float delta_time, uint32_t collision_steps);
KA_API uint32_t ka_world_body_count(void *world);
KA_API int ka_world_get_transform(void *world, uint32_t body_id, KAPhysicsTransform *out_transform);

#ifdef __cplusplus
}
#endif

#endif
