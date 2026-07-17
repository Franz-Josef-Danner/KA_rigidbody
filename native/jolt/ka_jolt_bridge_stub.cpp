#define KA_PHYSICS_BRIDGE_EXPORTS
#include "../ka_physics_bridge.h"

#include <string>

namespace {
thread_local std::string g_last_error =
    "Jolt world/body/step implementation is not linked in the 0.1.0 scaffold.";
}

extern "C" {

int ka_physics_abi_version(void) {
    return KA_PHYSICS_ABI_VERSION;
}

const char *ka_physics_backend_name(void) {
    return "Jolt bridge scaffold";
}

const char *ka_physics_last_error(void) {
    return g_last_error.c_str();
}

void *ka_world_create(KAPhysicsVec3, uint32_t) {
    return nullptr;
}

void ka_world_destroy(void *) {
}

int ka_world_step(void *, float, uint32_t) {
    return 0;
}

uint32_t ka_world_body_count(void *) {
    return 0;
}

int ka_world_get_transform(void *, uint32_t, KAPhysicsTransform *) {
    return 0;
}

}
