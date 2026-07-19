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

enum { KA_PHYSICS_ABI_VERSION = 2 };

typedef struct KAPhysicsVec3 { float x, y, z; } KAPhysicsVec3;
typedef struct KAPhysicsQuat { float w, x, y, z; } KAPhysicsQuat;
typedef struct KAPhysicsTransform { KAPhysicsVec3 position; KAPhysicsQuat rotation; } KAPhysicsTransform;

typedef enum KAPhysicsMotionType {
    KA_MOTION_STATIC = 0,
    KA_MOTION_KINEMATIC = 1,
    KA_MOTION_DYNAMIC = 2
} KAPhysicsMotionType;

typedef enum KAPhysicsShapeType {
    KA_SHAPE_BOX = 0,
    KA_SHAPE_SPHERE = 1,
    KA_SHAPE_PLANE = 2
} KAPhysicsShapeType;

typedef enum KAPhysicsContactEventType {
    KA_CONTACT_ADDED = 0,
    KA_CONTACT_PERSISTED = 1,
    KA_CONTACT_REMOVED = 2
} KAPhysicsContactEventType;

typedef struct KAPhysicsWorldDesc {
    KAPhysicsVec3 gravity;
    uint32_t max_bodies;
    uint32_t max_body_pairs;
    uint32_t max_contact_constraints;
    uint32_t temp_allocator_bytes;
    uint32_t worker_threads;
    float penetration_slop;
} KAPhysicsWorldDesc;

typedef struct KAPhysicsBodyDesc {
    KAPhysicsTransform transform;
    uint32_t motion_type;
    float mass;
    uint64_t user_data;
    uint32_t category;
    uint32_t mask;
    float friction;
    float restitution;
    float linear_damping;
    float angular_damping;
    uint32_t continuous_collision;
} KAPhysicsBodyDesc;

typedef struct KAPhysicsCompoundChild {
    KAPhysicsTransform local_transform;
    const KAPhysicsVec3 *vertices;
    uint32_t vertex_count;
    uint32_t user_data;
} KAPhysicsCompoundChild;

typedef struct KAPhysicsContactEvent {
    uint32_t body1;
    uint32_t body2;
    uint32_t event_type;
    KAPhysicsVec3 point;
    KAPhysicsVec3 normal;
    float impulse;
    float penetration;
} KAPhysicsContactEvent;

KA_API int ka_physics_abi_version(void);
KA_API const char *ka_physics_backend_name(void);
KA_API const char *ka_physics_backend_version(void);
KA_API uint64_t ka_physics_capabilities(void);
KA_API const char *ka_physics_last_error(void);

KA_API void *ka_world_create(const KAPhysicsWorldDesc *desc);
KA_API void ka_world_destroy(void *world);
KA_API int ka_world_step(void *world, float delta_time, uint32_t collision_steps);
KA_API uint32_t ka_world_body_count(void *world);

KA_API uint32_t ka_body_create_primitive(void *world, const KAPhysicsBodyDesc *desc, uint32_t shape_type, const float *size, uint32_t size_count);
KA_API uint32_t ka_body_create_convex(void *world, const KAPhysicsBodyDesc *desc, const KAPhysicsVec3 *vertices, uint32_t vertex_count);
KA_API uint32_t ka_body_create_mesh(void *world, const KAPhysicsBodyDesc *desc, const KAPhysicsVec3 *vertices, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count);
KA_API uint32_t ka_body_create_compound_convex(void *world, const KAPhysicsBodyDesc *desc, const KAPhysicsCompoundChild *children, uint32_t child_count);
KA_API int ka_body_destroy(void *world, uint32_t body_id);
KA_API int ka_body_get_transform(void *world, uint32_t body_id, KAPhysicsTransform *out_transform);
KA_API int ka_body_get_linear_velocity(void *world, uint32_t body_id, KAPhysicsVec3 *out_velocity);
KA_API int ka_body_get_angular_velocity(void *world, uint32_t body_id, KAPhysicsVec3 *out_velocity);
KA_API int ka_body_set_linear_velocity(void *world, uint32_t body_id, KAPhysicsVec3 velocity);
KA_API int ka_body_set_angular_velocity(void *world, uint32_t body_id, KAPhysicsVec3 velocity);
KA_API int ka_body_activate(void *world, uint32_t body_id);
KA_API int ka_body_deactivate(void *world, uint32_t body_id);
KA_API int ka_body_is_active(void *world, uint32_t body_id);

/* Drains up to capacity events. Call with out_events=NULL/capacity=0 to query
 * the number currently queued without consuming them. */
KA_API uint32_t ka_world_drain_contact_events(void *world, KAPhysicsContactEvent *out_events, uint32_t capacity);

#ifdef __cplusplus
}
#endif

#endif
