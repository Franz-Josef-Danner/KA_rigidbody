#define KA_PHYSICS_BRIDGE_EXPORTS
#include "../ka_physics_bridge.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

JPH_SUPPRESS_WARNINGS
using namespace JPH;

namespace {
thread_local std::string g_last_error;
std::once_flag g_jolt_init;

void set_error(const std::string &message) { g_last_error = message; }
void trace_impl(const char *fmt, ...) {
    char buffer[2048];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
}

void initialize_jolt() {
    std::call_once(g_jolt_init, [] {
        RegisterDefaultAllocator();
        Trace = trace_impl;
        Factory::sInstance = new Factory();
        RegisterTypes();
    });
}

namespace Layers {
static constexpr ObjectLayer NON_MOVING = 0;
static constexpr ObjectLayer MOVING = 1;
static constexpr ObjectLayer NUM_LAYERS = 2;
}
namespace BPLayers {
static constexpr BroadPhaseLayer NON_MOVING(0);
static constexpr BroadPhaseLayer MOVING(1);
static constexpr uint NUM_LAYERS = 2;
}

class BPLayerInterface final : public BroadPhaseLayerInterface {
public:
    uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM_LAYERS; }
    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer layer) const override {
        return layer == Layers::NON_MOVING ? BPLayers::NON_MOVING : BPLayers::MOVING;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(BroadPhaseLayer layer) const override {
        return layer == BPLayers::NON_MOVING ? "NON_MOVING" : "MOVING";
    }
#endif
};

class ObjectVsBPFilter final : public ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(ObjectLayer layer1, BroadPhaseLayer layer2) const override {
        return layer1 == Layers::MOVING || layer2 == BPLayers::MOVING;
    }
};

class ObjectPairFilter final : public ObjectLayerPairFilter {
public:
    bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override {
        return a == Layers::MOVING || b == Layers::MOVING;
    }
};

struct BodyFilterData { uint32_t category = 1; uint32_t mask = 0xffff; };

struct World;
class ContactCollector final : public ContactListener {
public:
    explicit ContactCollector(World *owner) : m_owner(owner) {}
    ValidateResult OnContactValidate(const Body &a, const Body &b, RVec3Arg, const CollideShapeResult &) override;
    void OnContactAdded(const Body &a, const Body &b, const ContactManifold &m, ContactSettings &) override;
    void OnContactPersisted(const Body &a, const Body &b, const ContactManifold &m, ContactSettings &) override;
    void OnContactRemoved(const SubShapeIDPair &pair) override;
private:
    void push(const BodyID &a, const BodyID &b, uint32_t type, const ContactManifold *m);
    World *m_owner;
};

struct World {
    explicit World(const KAPhysicsWorldDesc &d)
        : temp_allocator(std::max<uint32_t>(d.temp_allocator_bytes, 16u * 1024u * 1024u)),
          jobs(cMaxPhysicsJobs, cMaxPhysicsBarriers, std::max(1u, d.worker_threads)),
          contacts(this) {
        physics.Init(
            std::max(128u, d.max_bodies), 0,
            std::max(1024u, d.max_body_pairs),
            std::max(1024u, d.max_contact_constraints),
            bp_interface, object_vs_bp, object_pair);
        physics.SetGravity(Vec3(d.gravity.x, d.gravity.y, d.gravity.z));
        PhysicsSettings settings = physics.GetPhysicsSettings();
        settings.mPenetrationSlop = std::max(1.0e-6f, d.penetration_slop);
        physics.SetPhysicsSettings(settings);
        physics.SetContactListener(&contacts);
    }

    BodyInterface &bodies() { return physics.GetBodyInterface(); }

    TempAllocatorImpl temp_allocator;
    JobSystemThreadPool jobs;
    BPLayerInterface bp_interface;
    ObjectVsBPFilter object_vs_bp;
    ObjectPairFilter object_pair;
    PhysicsSystem physics;
    ContactCollector contacts;
    std::mutex mutex;
    std::unordered_map<uint32_t, BodyFilterData> filters;
    std::vector<KAPhysicsContactEvent> events;
};

BodyFilterData filter_for(World *w, const BodyID &id) {
    std::lock_guard<std::mutex> lock(w->mutex);
    auto it = w->filters.find(id.GetIndexAndSequenceNumber());
    return it == w->filters.end() ? BodyFilterData{} : it->second;
}

ContactListener::ValidateResult ContactCollector::OnContactValidate(
    const Body &a, const Body &b, RVec3Arg, const CollideShapeResult &) {
    const BodyFilterData fa = filter_for(m_owner, a.GetID());
    const BodyFilterData fb = filter_for(m_owner, b.GetID());
    const bool allowed = (fa.mask & fb.category) != 0 && (fb.mask & fa.category) != 0;
    return allowed ? ValidateResult::AcceptAllContactsForThisBodyPair : ValidateResult::RejectAllContactsForThisBodyPair;
}

void ContactCollector::push(const BodyID &a, const BodyID &b, uint32_t type, const ContactManifold *m) {
    KAPhysicsContactEvent e{};
    e.body1 = a.GetIndexAndSequenceNumber();
    e.body2 = b.GetIndexAndSequenceNumber();
    e.event_type = type;
    if (m != nullptr) {
        const Vec3 n = m->mWorldSpaceNormal;
        e.normal = {n.GetX(), n.GetY(), n.GetZ()};
        e.penetration = m->mPenetrationDepth;
        if (!m->mRelativeContactPointsOn1.empty()) {
            const RVec3 p = m->mBaseOffset + m->mRelativeContactPointsOn1[0];
            e.point = {float(p.GetX()), float(p.GetY()), float(p.GetZ())};
        }
    }
    std::lock_guard<std::mutex> lock(m_owner->mutex);
    m_owner->events.push_back(e);
}
void ContactCollector::OnContactAdded(const Body &a, const Body &b, const ContactManifold &m, ContactSettings &) {
    push(a.GetID(), b.GetID(), KA_CONTACT_ADDED, &m);
}
void ContactCollector::OnContactPersisted(const Body &a, const Body &b, const ContactManifold &m, ContactSettings &) {
    push(a.GetID(), b.GetID(), KA_CONTACT_PERSISTED, &m);
}
void ContactCollector::OnContactRemoved(const SubShapeIDPair &pair) {
    push(pair.GetBody1ID(), pair.GetBody2ID(), KA_CONTACT_REMOVED, nullptr);
}

Vec3 vec3(KAPhysicsVec3 v) { return Vec3(v.x, v.y, v.z); }
RVec3 rvec3(KAPhysicsVec3 v) { return RVec3(v.x, v.y, v.z); }
Quat quat(KAPhysicsQuat q) { return Quat(q.x, q.y, q.z, q.w).Normalized(); }

EMotionType motion_type(uint32_t value) {
    if (value == KA_MOTION_STATIC) return EMotionType::Static;
    if (value == KA_MOTION_KINEMATIC) return EMotionType::Kinematic;
    return EMotionType::Dynamic;
}

uint32_t add_body(World *w, const KAPhysicsBodyDesc &d, ShapeRefC shape) {
    if (shape == nullptr) { set_error("Shape creation returned null"); return BodyID::cInvalidBodyID; }
    const EMotionType motion = motion_type(d.motion_type);
    const ObjectLayer layer = motion == EMotionType::Static ? Layers::NON_MOVING : Layers::MOVING;
    BodyCreationSettings settings(shape, rvec3(d.transform.position), quat(d.transform.rotation), motion, layer);
    settings.mUserData = d.user_data;
    settings.mFriction = std::max(0.0f, d.friction);
    settings.mRestitution = std::clamp(d.restitution, 0.0f, 1.0f);
    settings.mLinearDamping = std::max(0.0f, d.linear_damping);
    settings.mAngularDamping = std::max(0.0f, d.angular_damping);
    settings.mMotionQuality = d.continuous_collision ? EMotionQuality::LinearCast : EMotionQuality::Discrete;
    if (motion == EMotionType::Dynamic && d.mass > 0.0f) {
        settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = d.mass;
    }
    BodyID id = w->bodies().CreateAndAddBody(settings, motion == EMotionType::Dynamic ? EActivation::Activate : EActivation::DontActivate);
    if (id.IsInvalid()) { set_error("Jolt could not allocate a body"); return BodyID::cInvalidBodyID; }
    {
        std::lock_guard<std::mutex> lock(w->mutex);
        w->filters[id.GetIndexAndSequenceNumber()] = {d.category, d.mask};
    }
    return id.GetIndexAndSequenceNumber();
}

ShapeRefC create_convex(const KAPhysicsVec3 *vertices, uint32_t count) {
    if (vertices == nullptr || count < 4) { set_error("Convex hull requires at least four vertices"); return nullptr; }
    Array<Vec3> points;
    points.reserve(count);
    for (uint32_t i = 0; i < count; ++i) points.push_back(vec3(vertices[i]));
    ConvexHullShapeSettings settings(points);
    // Preserve the authored sharp fracture silhouette. Jolt otherwise shrinks
    // the source hull and re-inflates it with the default convex radius, which
    // makes sharp render vertices appear below a plane even while the rounded
    // physical hull is resting correctly.
    settings.mMaxConvexRadius = 0.0f;
    auto result = settings.Create();
    if (result.HasError()) { set_error(result.GetError()); return nullptr; }
    return result.Get();
}

World *as_world(void *ptr) { return static_cast<World *>(ptr); }
BodyID body_id(uint32_t value) { return BodyID(value); }
}

extern "C" {
int ka_physics_abi_version(void) { return KA_PHYSICS_ABI_VERSION; }
const char *ka_physics_backend_name(void) { return "Jolt Physics native bridge"; }
const char *ka_physics_backend_version(void) { return "5.6.0"; }
uint64_t ka_physics_capabilities(void) { return 0x1full; }
const char *ka_physics_last_error(void) { return g_last_error.c_str(); }

void *ka_world_create(const KAPhysicsWorldDesc *desc) {
    if (desc == nullptr) { set_error("World descriptor is null"); return nullptr; }
    try { initialize_jolt(); return new World(*desc); }
    catch (...) { set_error("Unexpected failure while creating Jolt world"); return nullptr; }
}
void ka_world_destroy(void *world) { delete as_world(world); }
int ka_world_step(void *world, float dt, uint32_t collision_steps) {
    if (world == nullptr || dt < 0.0f) return 0;
    World *w = as_world(world);
    const EPhysicsUpdateError error = w->physics.Update(dt, std::max(1u, collision_steps), &w->temp_allocator, &w->jobs);
    if (error != EPhysicsUpdateError::None) { set_error("Jolt PhysicsSystem::Update reported an error"); return 0; }
    return 1;
}
uint32_t ka_world_body_count(void *world) { return world ? as_world(world)->physics.GetNumBodies() : 0; }

uint32_t ka_body_create_primitive(void *world, const KAPhysicsBodyDesc *desc, uint32_t shape_type, const float *size, uint32_t size_count) {
    if (!world || !desc || !size) return BodyID::cInvalidBodyID;
    ShapeRefC shape;
    if (shape_type == KA_SHAPE_BOX && size_count >= 3) shape = new BoxShape(Vec3(std::max(size[0], 1.0e-5f), std::max(size[1], 1.0e-5f), std::max(size[2], 1.0e-5f)));
    else if (shape_type == KA_SHAPE_SPHERE && size_count >= 1) shape = new SphereShape(std::max(size[0], 1.0e-5f));
    else if (shape_type == KA_SHAPE_PLANE) shape = new PlaneShape(Plane(Vec3::sAxisY(), 0.0f), nullptr, size_count ? std::max(size[0], 1000.0f) : 1000.0f);
    else { set_error("Unsupported primitive shape"); return BodyID::cInvalidBodyID; }
    return add_body(as_world(world), *desc, shape);
}

uint32_t ka_body_create_convex(void *world, const KAPhysicsBodyDesc *desc, const KAPhysicsVec3 *vertices, uint32_t count) {
    if (!world || !desc) return BodyID::cInvalidBodyID;
    return add_body(as_world(world), *desc, create_convex(vertices, count));
}

uint32_t ka_body_create_mesh(void *world, const KAPhysicsBodyDesc *desc, const KAPhysicsVec3 *vertices, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count) {
    if (!world || !desc || !vertices || !indices || vertex_count < 3 || index_count < 3 || index_count % 3 != 0) return BodyID::cInvalidBodyID;
    VertexList points;
    points.reserve(vertex_count);
    for (uint32_t i = 0; i < vertex_count; ++i) points.push_back(Float3(vertices[i].x, vertices[i].y, vertices[i].z));
    IndexedTriangleList triangles;
    triangles.reserve(index_count / 3);
    for (uint32_t i = 0; i < index_count; i += 3) triangles.emplace_back(indices[i], indices[i + 1], indices[i + 2]);
    MeshShapeSettings settings(std::move(points), std::move(triangles));
    auto result = settings.Create();
    if (result.HasError()) { set_error(result.GetError()); return BodyID::cInvalidBodyID; }
    return add_body(as_world(world), *desc, result.Get());
}

uint32_t ka_body_create_compound_convex(void *world, const KAPhysicsBodyDesc *desc, const KAPhysicsCompoundChild *children, uint32_t child_count) {
    if (!world || !desc || !children || child_count == 0) return BodyID::cInvalidBodyID;
    StaticCompoundShapeSettings compound;
    for (uint32_t i = 0; i < child_count; ++i) {
        ShapeRefC child = create_convex(children[i].vertices, children[i].vertex_count);
        if (child == nullptr) return BodyID::cInvalidBodyID;
        compound.AddShape(vec3(children[i].local_transform.position), quat(children[i].local_transform.rotation), child, children[i].user_data);
    }
    auto result = compound.Create();
    if (result.HasError()) { set_error(result.GetError()); return BodyID::cInvalidBodyID; }
    return add_body(as_world(world), *desc, result.Get());
}

int ka_body_destroy(void *world, uint32_t id_value) {
    if (!world) return 0;
    World *w = as_world(world); BodyID id = body_id(id_value);
    w->bodies().RemoveBody(id); w->bodies().DestroyBody(id);
    std::lock_guard<std::mutex> lock(w->mutex); w->filters.erase(id_value); return 1;
}
int ka_body_get_transform(void *world, uint32_t id_value, KAPhysicsTransform *out) {
    if (!world || !out) return 0;
    RVec3 p; Quat q; as_world(world)->bodies().GetPositionAndRotation(body_id(id_value), p, q);
    out->position = {float(p.GetX()), float(p.GetY()), float(p.GetZ())};
    out->rotation = {q.GetW(), q.GetX(), q.GetY(), q.GetZ()}; return 1;
}
int ka_body_get_linear_velocity(void *world, uint32_t id_value, KAPhysicsVec3 *out) {
    if (!world || !out) return 0; Vec3 v = as_world(world)->bodies().GetLinearVelocity(body_id(id_value)); *out = {v.GetX(), v.GetY(), v.GetZ()}; return 1;
}
int ka_body_get_angular_velocity(void *world, uint32_t id_value, KAPhysicsVec3 *out) {
    if (!world || !out) return 0; Vec3 v = as_world(world)->bodies().GetAngularVelocity(body_id(id_value)); *out = {v.GetX(), v.GetY(), v.GetZ()}; return 1;
}
int ka_body_set_linear_velocity(void *world, uint32_t id_value, KAPhysicsVec3 v) { if (!world) return 0; as_world(world)->bodies().SetLinearVelocity(body_id(id_value), vec3(v)); return 1; }
int ka_body_set_angular_velocity(void *world, uint32_t id_value, KAPhysicsVec3 v) { if (!world) return 0; as_world(world)->bodies().SetAngularVelocity(body_id(id_value), vec3(v)); return 1; }
int ka_body_activate(void *world, uint32_t id_value) { if (!world) return 0; as_world(world)->bodies().ActivateBody(body_id(id_value)); return 1; }
int ka_body_deactivate(void *world, uint32_t id_value) { if (!world) return 0; as_world(world)->bodies().DeactivateBody(body_id(id_value)); return 1; }
int ka_body_is_active(void *world, uint32_t id_value) { return world && as_world(world)->bodies().IsActive(body_id(id_value)); }

uint32_t ka_world_drain_contact_events(void *world, KAPhysicsContactEvent *out, uint32_t capacity) {
    if (!world) return 0; World *w = as_world(world); std::lock_guard<std::mutex> lock(w->mutex);
    if (out == nullptr || capacity == 0) return static_cast<uint32_t>(w->events.size());
    const uint32_t count = std::min<uint32_t>(capacity, static_cast<uint32_t>(w->events.size()));
    std::copy_n(w->events.begin(), count, out); w->events.erase(w->events.begin(), w->events.begin() + count); return count;
}
}
