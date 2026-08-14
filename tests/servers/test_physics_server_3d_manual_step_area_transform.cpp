/**************************************************************************/
/*  test_physics_server_3d_manual_step_area_transform.cpp                 */
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

#include "tests/test_macros.h"

#include "modules/modules_enabled.gen.h" // For MODULE_JOLT_PHYSICS_ENABLED.

TEST_FORCE_LINK(test_physics_server_3d_manual_step_area_transform)

#if !defined(PHYSICS_3D_DISABLED) && defined(MODULE_JOLT_PHYSICS_ENABLED)

#include "servers/physics_3d/physics_server_3d.h"

#include "modules/jolt_physics/jolt_physics_server_3d.h"

namespace TestPhysicsServer3DManualStepAreaTransform {

static const real_t DT = 1.0f / 60.0f;

static const Vector3 POSE_A = Vector3(4, 1.5f, 5);
static const Vector3 POSE_B = Vector3(31, 0.25f, 8);

// The manual-step kinematic path under test is Jolt's, and the doctest host always boots the
// default (GodotPhysics3D) backend regardless of --path. A privately-owned JoltPhysicsServer3D is
// therefore constructed here so the gate binds to Jolt in every run rather than skipping.
struct JoltHarness {
	JoltPhysicsServer3D *server = nullptr;

	JoltHarness() {
		server = memnew(JoltPhysicsServer3D(false));
		server->init();
		server->set_active(true);
	}

	~JoltHarness() {
		server->finish();
		memdelete(server);
	}
};

static Vector3 area_origin(PhysicsServer3D *ps, RID area) {
	return ps->area_get_transform(area).origin;
}

static RID make_manual_space(PhysicsServer3D *ps) {
	RID space = ps->space_create();
	ps->space_set_active(space, true);
	ps->space_set_stepping_mode(space, PS3DE::SPACE_STEPPING_MODE_MANUAL);
	return space;
}

static RID make_area(PhysicsServer3D *ps, RID space, RID shape, const Vector3 &at) {
	RID area = ps->area_create();
	ps->area_set_space(area, space);
	ps->area_add_shape(area, shape);
	ps->area_set_monitorable(area, true);
	ps->area_set_transform(area, Transform3D(Basis(), at));
	return area;
}

TEST_SUITE("[PhysicsServer3D][ManualStep] area transform latching") {
	// An area that is never moved must not drift: this isolates the teleport as the trigger
	// rather than manual stepping in general.
	TEST_CASE("[SceneTree][PhysicsServer3D] a stationary area keeps its pose across manual steps") {
		JoltHarness harness;
		PhysicsServer3D *ps = harness.server;
		REQUIRE(ps != nullptr);

		RID shape = ps->box_shape_create();
		ps->shape_set_data(shape, Vector3(0.6f, 1.0f, 0.6f));

		RID space = make_manual_space(ps);
		RID area = make_area(ps, space, shape, POSE_A);

		for (int i = 0; i < 32; i++) {
			ps->space_step_safe(space, DT);
			CHECK_MESSAGE(
					area_origin(ps, area).is_equal_approx(POSE_A),
					"A stationary area must hold its pose on every manual step.");
		}

		ps->free_rid(area);
		ps->free_rid(space);
		ps->free_rid(shape);
	}

	// The kill-gate: one absolute teleport, then the pose must stay put forever.
	//
	// Before the fix the physics-server pose cycled between POSE_B, POSE_B + (POSE_B - POSE_A)
	// and POSE_A, because JoltArea3D::advance_kinematic_manual_step() returned early once the
	// target was reached without clearing the kinematic velocity that carried the area there,
	// so PhysicsSystem::Update kept integrating it.
	TEST_CASE("[SceneTree][PhysicsServer3D] an area teleported once stays at the requested pose") {
		JoltHarness harness;
		PhysicsServer3D *ps = harness.server;
		REQUIRE(ps != nullptr);

		RID shape = ps->box_shape_create();
		ps->shape_set_data(shape, Vector3(0.6f, 1.0f, 0.6f));

		RID space = make_manual_space(ps);
		RID area = make_area(ps, space, shape, POSE_A);

		ps->space_step_safe(space, DT);
		REQUIRE(area_origin(ps, area).is_equal_approx(POSE_A));

		ps->area_set_transform(area, Transform3D(Basis(), POSE_B));

		// The teleport is allowed to take one step to be applied; from then on it must hold.
		ps->space_step_safe(space, DT);

		const Vector3 overshoot = POSE_B + (POSE_B - POSE_A);
		for (int i = 0; i < 32; i++) {
			const Vector3 origin = area_origin(ps, area);

			CHECK_FALSE_MESSAGE(
					origin.is_equal_approx(POSE_A),
					"A teleported area must never fall back to the pose it was moved away from.");
			CHECK_FALSE_MESSAGE(
					origin.is_equal_approx(overshoot),
					"A teleported area must never overshoot by the teleport delta — the kinematic "
					"velocity that carried it to the target was not cleared.");
			CHECK_MESSAGE(
					origin.is_equal_approx(POSE_B),
					"A teleported area must hold the requested absolute pose on every later step.");

			ps->space_step_safe(space, DT);
		}

		ps->free_rid(area);
		ps->free_rid(space);
		ps->free_rid(shape);
	}

	// The overlap set is what gameplay reads; a drifting collider corrupts it even when the
	// reported pose happens to be correct on the sampled step.
	TEST_CASE("[SceneTree][PhysicsServer3D] a teleported area keeps a resting body out of its overlaps") {
		JoltHarness harness;
		PhysicsServer3D *ps = harness.server;
		REQUIRE(ps != nullptr);

		RID area_shape = ps->box_shape_create();
		ps->shape_set_data(area_shape, Vector3(0.6f, 1.0f, 0.6f));
		RID body_shape = ps->box_shape_create();
		ps->shape_set_data(body_shape, Vector3(0.4f, 0.9f, 0.4f));

		RID space = make_manual_space(ps);
		RID area = make_area(ps, space, area_shape, POSE_A);

		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, body_shape);
		ps->body_set_mode(body, PS3DE::BODY_MODE_STATIC);
		ps->body_set_state(body, PS3DE::BODY_STATE_TRANSFORM, Transform3D(Basis(), POSE_A));

		ps->space_step_safe(space, DT);

		// Move the area far away from the resting body and keep stepping.
		ps->area_set_transform(area, Transform3D(Basis(), POSE_B));
		ps->space_step_safe(space, DT);

		for (int i = 0; i < 32; i++) {
			CHECK_MESSAGE(
					area_origin(ps, area).distance_to(POSE_A) > 1.0f,
					"An area moved away from a resting body must not return to it on any later step.");
			ps->space_step_safe(space, DT);
		}

		ps->free_rid(body);
		ps->free_rid(area);
		ps->free_rid(space);
		ps->free_rid(area_shape);
		ps->free_rid(body_shape);
	}
}

} // namespace TestPhysicsServer3DManualStepAreaTransform

#endif // PHYSICS_3D_DISABLED
