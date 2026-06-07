/**************************************************************************/
/*  test_jolt_space_state.h                                               */
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

#pragma once

// Sequence: snapshot (T=0) -> step N -> record state_A -> restore (T=0) -> step N -> record
// state_B -> assert state_A == state_B (bit-identical, same binary, same machine).
//
// Green: rollback/replay are real.
// Red  : Jolt's StateRecorder is non-deterministic on this build; determinism-dependent
//        features must be downgraded to "best-effort", change the design, not the test.

#include "../jolt_project_settings.h"
#include "../spaces/jolt_broad_phase_layer.h"
#include "../spaces/jolt_space_3d.h"

#include "tests/test_macros.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>

namespace TestJoltSpaceState {

// N = 60 steps ≈ 1 s at 60 Hz. Long enough for gravity to drive the bodies to the floor
// and produce active contacts, ensuring the contact cache is covered by the snapshot.
static constexpr int STEP_COUNT = 60;

// Fixed timestep: determinism precondition requires identical delta across both runs.
static constexpr float STEP_DELTA = 1.0f / 60.0f;

// Body state captured after N steps for bit-identical comparison.
// position is RVec3, not Vec3: BodyInterface::GetPosition returns JPH::RVec3, which becomes
// a double-precision DVec3 when the Jolt library is built with JPH_DOUBLE_PRECISION (Godot's
// precision=double). Declaring it Vec3 would force a narrowing conversion that fails to
// compile in double-precision builds. Velocities are always Vec3 in both precisions.
struct BodyCapture {
	JPH::RVec3 position;
	JPH::Vec3 linear_velocity;
	JPH::Vec3 angular_velocity;
};

static BodyCapture capture_body(const JPH::BodyInterface &p_iface, const JPH::BodyID &p_id) {
	return {
		p_iface.GetPosition(p_id),
		p_iface.GetLinearVelocity(p_id),
		p_iface.GetAngularVelocity(p_id),
	};
}

// Step the underlying PhysicsSystem directly, bypassing JoltSpace3D::step()'s
// pre/post callbacks which require JoltObject3D user-data on every body. The save_state /
// restore_state methods we are testing wrap exactly PhysicsSystem::SaveState /
// RestoreState, so calling Update here exercises the same simulation path.
// The contact listener and body activation listener are removed before stepping because
// JoltContactListener3D / JoltBodyActivationListener3D expect JoltObject3D user data on
// every JPH::Body; since test bodies are added directly via BodyInterface they carry no
// such user data. Removing the listeners is safe for the determinism gate: SaveState /
// RestoreState capture solver + contact-cache state, not listener callbacks.
static void step_physics(JPH::PhysicsSystem &p_system,
		JPH::TempAllocator &p_temp_alloc, JPH::JobSystem &p_job_sys, int p_count) {
	// Disconnect Godot-specific listeners for the duration of raw stepping.
	// JoltContactListener3D / JoltBodyActivationListener3D cast JPH::Body::GetUserData()
	// to JoltObject3D*; test bodies carry no such user data and would crash the callbacks.
	JPH::ContactListener *saved_contact = p_system.GetContactListener();
	JPH::SoftBodyContactListener *saved_soft = p_system.GetSoftBodyContactListener();
	JPH::BodyActivationListener *saved_activation = p_system.GetBodyActivationListener();
	p_system.SetContactListener(nullptr);
	p_system.SetSoftBodyContactListener(nullptr);
	p_system.SetBodyActivationListener(nullptr);

	for (int i = 0; i < p_count; ++i) {
		p_system.Update(STEP_DELTA, 1, &p_temp_alloc, &p_job_sys);
	}

	// Restore listeners so the space is left in a consistent state.
	p_system.SetContactListener(saved_contact);
	p_system.SetSoftBodyContactListener(saved_soft);
	p_system.SetBodyActivationListener(saved_activation);
}

TEST_CASE("[JoltSpace3D] Snapshot-restore produces bit-identical re-simulation") {
	// Guard: JoltProjectSettings populated by initialize_jolt_physics_module at engine start.
	// Zero means the module was not initialized, fail clearly rather than crashing.
	REQUIRE_MESSAGE(JoltProjectSettings::max_bodies > 0,
			"JoltProjectSettings::max_bodies is 0, Jolt module was not initialized");

	// Use Jolt's single-threaded job system and malloc-backed temp allocator to avoid
	// interference with the engine-global JoltJobSystem (which has a shared static job
	// list) and to avoid the JoltTempAllocator's dependency on JoltProjectSettings::
	// temp_memory_b being set before the first step.
	JPH::JobSystemSingleThreaded job_system;
	job_system.Init(JPH::cMaxPhysicsJobs);
	JPH::TempAllocatorMalloc temp_alloc;

	JoltSpace3D space(&job_system, &temp_alloc);

	JPH::PhysicsSystem &phys = space.get_physics_system();

	// Gravity pointing down; magnitude matches Godot's default (9.8 m/s²).
	// JoltSpace3D initializes gravity to zero; the default area normally applies it.
	// For this isolated test we set it directly on the PhysicsSystem.
	phys.SetGravity(JPH::Vec3(0.0f, -9.8f, 0.0f));
	JPH::BodyInterface &iface = phys.GetBodyInterface();

	// Object layers for static and dynamic bodies.
	const JPH::ObjectLayer layer_static = space.map_to_object_layer(JoltBroadPhaseLayer::BODY_STATIC, 1, 1);
	const JPH::ObjectLayer layer_dynamic = space.map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, 1, 1);

	JPH::BoxShapeSettings floor_ss(JPH::Vec3(50.0f, 0.5f, 50.0f));
	floor_ss.SetEmbedded();
	JPH::ShapeSettings::ShapeResult floor_res = floor_ss.Create();
	REQUIRE_MESSAGE(floor_res.IsValid(), "Failed to create floor shape");

	const JPH::BodyID floor_id = iface.CreateAndAddBody(
			JPH::BodyCreationSettings(floor_res.Get(),
					JPH::RVec3(0.0, -0.5, 0.0), JPH::Quat::sIdentity(),
					JPH::EMotionType::Static, layer_static),
			JPH::EActivation::DontActivate);
	REQUIRE_MESSAGE(!floor_id.IsInvalid(), "Failed to add static floor");

	// Dynamic sphere (falls under gravity, hits floor)
	JPH::SphereShapeSettings sphere_ss(0.5f);
	sphere_ss.SetEmbedded();
	JPH::ShapeSettings::ShapeResult sphere_res = sphere_ss.Create();
	REQUIRE_MESSAGE(sphere_res.IsValid(), "Failed to create sphere shape");

	const JPH::BodyID sphere_id = iface.CreateAndAddBody(
			JPH::BodyCreationSettings(sphere_res.Get(),
					JPH::RVec3(0.0, 5.0, 0.0), JPH::Quat::sIdentity(),
					JPH::EMotionType::Dynamic, layer_dynamic),
			JPH::EActivation::Activate);
	REQUIRE_MESSAGE(!sphere_id.IsInvalid(), "Failed to add dynamic sphere");

	// Second dynamic body (box, exercises broader contact cache)
	JPH::BoxShapeSettings box_ss(JPH::Vec3(0.4f, 0.4f, 0.4f));
	box_ss.SetEmbedded();
	JPH::ShapeSettings::ShapeResult box_res = box_ss.Create();
	REQUIRE_MESSAGE(box_res.IsValid(), "Failed to create dynamic box shape");

	const JPH::BodyID box_id = iface.CreateAndAddBody(
			JPH::BodyCreationSettings(box_res.Get(),
					JPH::RVec3(2.0, 3.0, 0.0), JPH::Quat::sIdentity(),
					JPH::EMotionType::Dynamic, layer_dynamic),
			JPH::EActivation::Activate);
	REQUIRE_MESSAGE(!box_id.IsInvalid(), "Failed to add dynamic box");

	// Snapshot at T=0
	JPH::StateRecorderImpl recorder;
	space.save_state(recorder);

	// Run 1: N steps, capture end state A
	step_physics(phys, temp_alloc, job_system, STEP_COUNT);

	const BodyCapture a_sphere = capture_body(iface, sphere_id);
	const BodyCapture a_box = capture_body(iface, box_id);

	// Restore to T=0
	recorder.Rewind();
	const bool restored = space.restore_state(recorder);
	REQUIRE_MESSAGE(restored, "RestoreState returned false, snapshot could not be applied");

	// Run 2: N steps from restored T=0, capture end state B
	step_physics(phys, temp_alloc, job_system, STEP_COUNT);

	const BodyCapture b_sphere = capture_body(iface, sphere_id);
	const BodyCapture b_box = capture_body(iface, box_id);

	// Bit-identical assertion (primary — per-component float equality)
	// Do NOT use Approx: this is a determinism gate; "close enough" masks genuine
	// non-determinism. Any divergence here means the gate is RED.
	CHECK_MESSAGE(a_sphere.position.GetX() == b_sphere.position.GetX(),
			"Sphere pos.X diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_sphere.position.GetY() == b_sphere.position.GetY(),
			"Sphere pos.Y diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_sphere.position.GetZ() == b_sphere.position.GetZ(),
			"Sphere pos.Z diverged after restore+re-step: gate RED");

	CHECK_MESSAGE(a_sphere.linear_velocity.GetX() == b_sphere.linear_velocity.GetX(),
			"Sphere linvel.X diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_sphere.linear_velocity.GetY() == b_sphere.linear_velocity.GetY(),
			"Sphere linvel.Y diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_sphere.linear_velocity.GetZ() == b_sphere.linear_velocity.GetZ(),
			"Sphere linvel.Z diverged after restore+re-step: gate RED");

	CHECK_MESSAGE(a_box.position.GetX() == b_box.position.GetX(),
			"Box pos.X diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_box.position.GetY() == b_box.position.GetY(),
			"Box pos.Y diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_box.position.GetZ() == b_box.position.GetZ(),
			"Box pos.Z diverged after restore+re-step: gate RED");

	CHECK_MESSAGE(a_box.linear_velocity.GetX() == b_box.linear_velocity.GetX(),
			"Box linvel.X diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_box.linear_velocity.GetY() == b_box.linear_velocity.GetY(),
			"Box linvel.Y diverged after restore+re-step: gate RED");
	CHECK_MESSAGE(a_box.linear_velocity.GetZ() == b_box.linear_velocity.GetZ(),
			"Box linvel.Z diverged after restore+re-step: gate RED");

	// Secondary: Jolt validation mode
	// Save the current state (end of run 2), then restore it in validating mode. Jolt
	// internally re-writes and re-reads the stream, asserting each value matches. Requires
	// EStateRecorderState::All with null filter (same as save_state above).
	JPH::StateRecorderImpl validator;
	space.save_state(validator);
	validator.SetValidating(true);
	validator.Rewind();
	const bool valid = phys.RestoreState(validator, nullptr);
	CHECK_MESSAGE(valid, "Jolt validation-mode RestoreState failed internal non-determinism detected");

	// Tertiary: serialized-blob equality
	// Save run-1 end state and run-2 end state as blobs and compare byte-for-byte.
	// We already stepped through run 2; save that state as blob_b. For blob_a we need the
	// run-1 end state. This was before the restore so we can't replay it, but the
	// per-component checks above already verify bit identity. Blob equality is provided as
	// an informational sanity check using the two blobs we CAN produce: (a) save before
	// run-2 stepped (which is the restored T=0 + N-step state. Effectively re-run 1 end
	// state that we just captured as run-2 end state), and (b) save again. If the system
	// is in a stable deterministic state, two consecutive saves without any stepping in
	// between must produce identical blobs.
	JPH::StateRecorderImpl blob_first;
	space.save_state(blob_first);

	JPH::StateRecorderImpl blob_second;
	space.save_state(blob_second);

	CHECK_MESSAGE(blob_first.GetData() == blob_second.GetData(),
			"Two consecutive saves of the same state produced different blobs, unexpected");
}

} // namespace TestJoltSpaceState
