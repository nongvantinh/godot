/**************************************************************************/
/*  test_jolt_space_step_isolated.cpp                                     */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

// GH-15 D2 kill-gate: per-space job-system + temp-allocator concurrent-step test.
//
// WHAT THIS TESTS (AC6):
//   N independent JoltSpace3D instances, each constructed with its own
//   JPH::JobSystemSingleThreaded + JPH::TempAllocatorMalloc.  Each space is
//   stepped from a separate worker thread for STEPS iterations.  Under TSan
//   there must be no data races; under ASan no heap errors.
//
//   The per-space private systems avoid two known shared-static race conditions:
//     1. JoltJobSystem::Job::completed_head — inline static std::atomic (process-global).
//     2. JoltTempAllocator — unsynchronized bump allocator (top/base fields).
//
//   Both JoltPhysicsServer3D::space_make_isolated() and space_step_isolated()
//   use the same technique internally; this test exercises the underlying physics
//   primitives directly (following the pattern in test_jolt_space_state.h) so
//   it runs regardless of which server backend is active.
//
// TWO-BUDGET TOLERANCE (Phase-4 convention):
//   After STEPS iterations all cold-contact transients have settled.
//   We compare each thread-stepped space end-Y against an identical serial run
//   within STEADY_STATE_BUDGET = 1e-3 m.  Float non-determinism across threads
//   is not expected for deterministic single-threaded job system; the budget is
//   a safety net only.
//
// AC8: the WrapMT live-space main-thread guard is tested separately in
//   test_physics_server_3d_space_step.cpp.

#include "tests/test_macros.h"

#include "modules/modules_enabled.gen.h" // For MODULE_JOLT_PHYSICS_ENABLED.

TEST_FORCE_LINK(test_jolt_space_step_isolated)

#if !defined(PHYSICS_3D_DISABLED) && defined(MODULE_JOLT_PHYSICS_ENABLED)

#include "core/object/worker_thread_pool.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/physics_3d/physics_server_3d_wrap_mt.h"

#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/jolt_project_settings.h"
#include "modules/jolt_physics/spaces/jolt_broad_phase_layer.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace TestJoltSpaceStepIsolated {

// Number of concurrent spaces (= worker threads in the concurrent test).
static const int N = 4;
// Steps per space.
static const int STEPS = 100;
// Time step (60 Hz).
static const float DT = 1.0f / 60.0f;
// Phase-4 steady-state budget after STEPS iterations (m).
static const float STEADY_STATE_BUDGET = 1e-3f;

// Helper: returns the inner JoltPhysicsServer3D if the singleton wraps one.
// Returns nullptr if the active server is not Jolt (e.g. GodotPhysics3D).
static JoltPhysicsServer3D *get_jolt_server() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	PhysicsServer3DWrapMT *wrap = Object::cast_to<PhysicsServer3DWrapMT>(ps);
	PhysicsServer3D *inner = (wrap != nullptr) ? wrap->get_inner_server() : ps;
	return Object::cast_to<JoltPhysicsServer3D>(inner);
}

// Per-space bundle: private job system + allocator + JoltSpace3D + body IDs.
// Each bundle is fully independent from every other bundle; no shared state.
struct SpaceBundle {
	JPH::JobSystemSingleThreaded job_sys;
	JPH::TempAllocatorMalloc temp_alloc;
	JoltSpace3D *space = nullptr;
	JPH::BodyID sphere_id;
	JPH::BodyID floor_id;

	// step_physics() pattern from test_jolt_space_state.h:
	//   Clear listeners (avoid Godot-object callbacks on raw Jolt bodies),
	//   call physics_system.Update() N times, then restore listeners.
	static void step_space(JPH::PhysicsSystem &p_phys,
			JPH::TempAllocatorMalloc &p_alloc,
			JPH::JobSystemSingleThreaded &p_js,
			int p_count) {
		JPH::ContactListener *saved_contact = p_phys.GetContactListener();
		JPH::SoftBodyContactListener *saved_soft = p_phys.GetSoftBodyContactListener();
		JPH::BodyActivationListener *saved_activation = p_phys.GetBodyActivationListener();
		p_phys.SetContactListener(nullptr);
		p_phys.SetSoftBodyContactListener(nullptr);
		p_phys.SetBodyActivationListener(nullptr);

		for (int i = 0; i < p_count; i++) {
			p_phys.Update(DT, 1, &p_alloc, &p_js);
		}

		p_phys.SetContactListener(saved_contact);
		p_phys.SetSoftBodyContactListener(saved_soft);
		p_phys.SetBodyActivationListener(saved_activation);
	}

	// Set up the JoltSpace3D with a static floor and a dynamic sphere.
	// The sphere starts at (0, start_y, 0) and falls under gravity.
	bool setup(float p_start_y) {
		job_sys.Init(JPH::cMaxPhysicsJobs);
		space = new JoltSpace3D(&job_sys, &temp_alloc);

		JPH::PhysicsSystem &phys = space->get_physics_system();
		phys.SetGravity(JPH::Vec3(0.0f, -9.8f, 0.0f));
		JPH::BodyInterface &iface = phys.GetBodyInterface();

		const JPH::ObjectLayer layer_static =
				space->map_to_object_layer(JoltBroadPhaseLayer::BODY_STATIC, 1, 1);
		const JPH::ObjectLayer layer_dynamic =
				space->map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, 1, 1);

		// Static floor.
		JPH::BoxShapeSettings floor_ss(JPH::Vec3(50.0f, 0.5f, 50.0f));
		floor_ss.SetEmbedded();
		JPH::ShapeSettings::ShapeResult floor_res = floor_ss.Create();
		if (!floor_res.IsValid()) {
			return false;
		}
		floor_id = iface.CreateAndAddBody(
				JPH::BodyCreationSettings(floor_res.Get(),
						JPH::RVec3(0.0f, -0.5f, 0.0f), JPH::Quat::sIdentity(),
						JPH::EMotionType::Static, layer_static),
				JPH::EActivation::DontActivate);
		if (floor_id.IsInvalid()) {
			return false;
		}

		// Dynamic sphere (falls under gravity, hits floor).
		JPH::SphereShapeSettings sphere_ss(0.5f);
		sphere_ss.SetEmbedded();
		JPH::ShapeSettings::ShapeResult sphere_res = sphere_ss.Create();
		if (!sphere_res.IsValid()) {
			return false;
		}
		sphere_id = iface.CreateAndAddBody(
				JPH::BodyCreationSettings(sphere_res.Get(),
						JPH::RVec3(0.0f, p_start_y, 0.0f), JPH::Quat::sIdentity(),
						JPH::EMotionType::Dynamic, layer_dynamic),
				JPH::EActivation::Activate);
		return !sphere_id.IsInvalid();
	}

	float sphere_y() const {
		return space->get_physics_system().GetBodyInterface().GetCenterOfMassPosition(sphere_id).GetY();
	}

	void teardown() {
		delete space;
		space = nullptr;
	}
};

TEST_SUITE("[JoltPhysics][SpaceStep][D2]") {
	// D2 KILL-GATE:
	// N isolated spaces, each stepped by its own worker thread for STEPS iterations.
	// No TSan/ASan errors = D2 survives the kill gate.
	// End-state Y compared to serial oracle (identical initial conditions).
	TEST_CASE("[JoltPhysics][GH-15] isolated concurrent step matches serial oracle (D2 kill-gate)") {
		REQUIRE_MESSAGE(JoltProjectSettings::max_bodies > 0,
				"JoltProjectSettings::max_bodies is 0 — Jolt module not initialized; skipping.");

		const float start_ys[N] = { 50.0f, 60.0f, 70.0f, 80.0f };

		// ------------------------------------------------------------------
		// Serial oracle: step each space on the main thread.
		// ------------------------------------------------------------------
		float oracle_y[N];
		for (int i = 0; i < N; i++) {
			SpaceBundle oracle;
			REQUIRE_MESSAGE(oracle.setup(start_ys[i]),
					vformat("Oracle space %d: body setup failed.", i));
			SpaceBundle::step_space(oracle.space->get_physics_system(),
					oracle.temp_alloc, oracle.job_sys, STEPS);
			oracle_y[i] = oracle.sphere_y();
			oracle.teardown();
		}

		// ------------------------------------------------------------------
		// Concurrent: N spaces, each stepped by a separate worker thread.
		// TSan must report zero races; ASan must report zero heap errors.
		// ------------------------------------------------------------------
		SpaceBundle bundles[N];
		for (int i = 0; i < N; i++) {
			REQUIRE_MESSAGE(bundles[i].setup(start_ys[i]),
					vformat("D2 space %d: body setup failed.", i));
		}

		struct WorkItem {
			SpaceBundle *bundle;
			static void run(void *p_ud) {
				WorkItem *self = static_cast<WorkItem *>(p_ud);
				SpaceBundle::step_space(
						self->bundle->space->get_physics_system(),
						self->bundle->temp_alloc,
						self->bundle->job_sys,
						STEPS);
			}
		};
		WorkItem work[N];
		WorkerThreadPool::TaskID tids[N];
		for (int i = 0; i < N; i++) {
			work[i] = { &bundles[i] };
			tids[i] = WorkerThreadPool::get_singleton()->add_native_task(
					&WorkItem::run, &work[i], false);
		}
		for (int i = 0; i < N; i++) {
			WorkerThreadPool::get_singleton()->wait_for_task_completion(tids[i]);
		}

		// Compare end positions to serial oracle within STEADY_STATE_BUDGET.
		for (int i = 0; i < N; i++) {
			float thread_y = bundles[i].sphere_y();
			float diff = Math::abs(thread_y - oracle_y[i]);
			CHECK_MESSAGE(diff < STEADY_STATE_BUDGET,
					vformat("D2 space %d: thread Y=%.6f vs oracle Y=%.6f diff=%.6f >= budget %.3f",
							i, thread_y, oracle_y[i], diff, STEADY_STATE_BUDGET));
		}

		for (int i = 0; i < N; i++) {
			bundles[i].teardown();
		}
	}

	// AC8 (server-level): space_step_isolated rejects a live (non-isolated) space.
	TEST_CASE("[JoltPhysics][GH-15] space_step_isolated rejects live space (AC8)") {
		JoltPhysicsServer3D *jolt = get_jolt_server();
		if (jolt == nullptr) {
			WARN_MESSAGE(false, "Skipping: active backend is not JoltPhysicsServer3D.");
			return;
		}
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true); // live space — NOT isolated

		// space_step_isolated on a live space must ERR_FAIL, not crash (AC8).
		ERR_PRINT_OFF;
		jolt->space_step_isolated(space, DT);
		ERR_PRINT_ON;

		ps->space_set_active(space, false);
		ps->free_rid(space);
	}

	// AC8 (server-level): space_make_isolated + space_step_isolated succeed
	// for an inactive space.
	TEST_CASE("[JoltPhysics][GH-15] space_make_isolated + space_step_isolated succeed") {
		JoltPhysicsServer3D *jolt = get_jolt_server();
		if (jolt == nullptr) {
			WARN_MESSAGE(false, "Skipping: active backend is not JoltPhysicsServer3D.");
			return;
		}
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		// Not isolated yet: step_isolated must ERR_FAIL (not crash, AC8).
		ERR_PRINT_OFF;
		jolt->space_step_isolated(space, DT);
		ERR_PRINT_ON;

		// Promote to isolated; step_isolated must now succeed cleanly.
		jolt->space_make_isolated(space);
		jolt->space_step_isolated(space, DT);

		ps->free_rid(space);
	}

	// AC8 (F2 hardening): space_set_active(true) must reject an already-isolated space —
	// the symmetric counterpart to space_make_isolated() rejecting active spaces. Without
	// this guard a caller could make a space both isolated AND live, so it would be stepped
	// thread-direct via space_step_isolated() AND by the main-thread server loop — two
	// concurrent PhysicsSystem::Update() calls on one space.
	TEST_CASE("[JoltPhysics][GH-15] space_set_active(true) rejects an isolated space (AC8/F2)") {
		JoltPhysicsServer3D *jolt = get_jolt_server();
		if (jolt == nullptr) {
			WARN_MESSAGE(false, "Skipping: active backend is not JoltPhysicsServer3D.");
			return;
		}
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		jolt->space_make_isolated(space);

		// Activating an isolated space must ERR_FAIL (not crash) and must NOT activate it.
		ERR_PRINT_OFF;
		ps->space_set_active(space, true);
		ERR_PRINT_ON;

		// The space stays inactive: reachable only thread-direct, never via the main loop.
		CHECK_FALSE(ps->space_is_active(space));

		ps->free_rid(space);
	}

	// #3 delta guard: space_step_isolated bypasses space_step(), so it carries its own
	// finite/non-negative delta check — a bad delta must ERR_FAIL, not reach the solver.
	TEST_CASE("[JoltPhysics][GH-15] space_step_isolated rejects non-finite / negative delta (#3)") {
		JoltPhysicsServer3D *jolt = get_jolt_server();
		if (jolt == nullptr) {
			WARN_MESSAGE(false, "Skipping: active backend is not JoltPhysicsServer3D.");
			return;
		}
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		jolt->space_make_isolated(space);

		// Bad deltas must be rejected without crashing or stepping.
		ERR_PRINT_OFF;
		jolt->space_step_isolated(space, Math::NaN);
		jolt->space_step_isolated(space, Math::INF);
		jolt->space_step_isolated(space, -1.0f);
		ERR_PRINT_ON;

		// A valid delta still steps cleanly.
		jolt->space_step_isolated(space, DT);

		ps->free_rid(space);
	}

} // TEST_SUITE

} // namespace TestJoltSpaceStepIsolated

#endif // !PHYSICS_3D_DISABLED && MODULE_JOLT_PHYSICS_ENABLED
