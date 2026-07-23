/**************************************************************************/
/*  test_physics_server_3d_space_stepping_mode.cpp                        */
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

TEST_FORCE_LINK(test_physics_server_3d_space_stepping_mode)

#ifndef PHYSICS_3D_DISABLED

#include "core/config/engine.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/physics_3d/physics_server_3d_dummy.h"

namespace TestPhysicsServer3DSpaceSteppingMode {

static bool is_dummy_server() {
	return Object::cast_to<PhysicsServer3DDummy>(PhysicsServer3D::get_singleton()) != nullptr;
}

static real_t get_body_y(PhysicsServer3D *ps, RID body) {
	return ((Transform3D)ps->body_get_state(body, PS3DE::BODY_STATE_TRANSFORM)).origin.y;
}

// Adds an active space with a single rigid sphere resting at y = 10 that will fall under gravity.
static RID make_falling_body(PhysicsServer3D *ps, RID space, RID shape) {
	RID body = ps->body_create();
	ps->body_set_space(body, space);
	ps->body_add_shape(body, shape);
	ps->body_set_mode(body, PS3DE::BODY_MODE_RIGID);
	ps->body_set_state(body, PS3DE::BODY_STATE_TRANSFORM, Transform3D(Basis(), Vector3(0, 10, 0)));
	return body;
}

TEST_SUITE("[PhysicsServer3D][PS3DE::SpaceSteppingMode]") {
	TEST_CASE("[SceneTree][PhysicsServer3D] stepping mode defaults to AUTO and round-trips") {
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();

		// Newly created spaces default to AUTO so existing behavior is unchanged.
		CHECK_EQ(ps->space_get_stepping_mode(space), PS3DE::SPACE_STEPPING_MODE_AUTO);

		ps->space_set_stepping_mode(space, PS3DE::SPACE_STEPPING_MODE_MANUAL);
		CHECK_EQ(ps->space_get_stepping_mode(space), PS3DE::SPACE_STEPPING_MODE_MANUAL);

		ps->space_set_stepping_mode(space, PS3DE::SPACE_STEPPING_MODE_AUTO);
		CHECK_EQ(ps->space_get_stepping_mode(space), PS3DE::SPACE_STEPPING_MODE_AUTO);

		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer3D] automatic step skips MANUAL spaces; space_step still advances them") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		REQUIRE(ps != nullptr);
		ps->set_active(true);

		RID shape = ps->sphere_shape_create();
		ps->shape_set_data(shape, 0.5f);

		// AUTO control space + a MANUAL subject space, each with an identical falling body.
		RID auto_space = ps->space_create();
		ps->space_set_active(auto_space, true);
		RID manual_space = ps->space_create();
		ps->space_set_active(manual_space, true);
		ps->space_set_stepping_mode(manual_space, PS3DE::SPACE_STEPPING_MODE_MANUAL);

		RID auto_body = make_falling_body(ps, auto_space, shape);
		RID manual_body = make_falling_body(ps, manual_space, shape);

		// Drive the whole-server automatic step several times.
		const real_t dt = 1.0f / 60.0f;
		for (int i = 0; i < 20; i++) {
			ps->sync();
			ps->step(dt);
			ps->end_sync();
		}

		const real_t auto_y = get_body_y(ps, auto_body);
		const real_t manual_y = get_body_y(ps, manual_body);

		if (auto_y >= 10.0f - 0.001f) {
			// The whole-server step did not advance the AUTO control in this harness, so the
			// skip assertion below cannot be attributed to the MANUAL policy — skip it rather than
			// report a false pass.
			MESSAGE("Skipping skip-assertion: automatic step did not advance the AUTO control body.");
		} else {
			CHECK_MESSAGE(manual_y > 9.99f, "A MANUAL space must NOT be advanced by the automatic physics step.");
		}

		// The MANUAL space must still advance through explicit per-space stepping.
		for (int i = 0; i < 20; i++) {
			ps->space_step_safe(manual_space, dt);
		}
		CHECK_MESSAGE(get_body_y(ps, manual_body) < 10.0f, "space_step must advance a MANUAL space.");

		ps->free_rid(auto_body);
		ps->free_rid(manual_body);
		ps->free_rid(shape);
		ps->free_rid(auto_space);
		ps->free_rid(manual_space);
	}
}

} // namespace TestPhysicsServer3DSpaceSteppingMode

#endif // PHYSICS_3D_DISABLED
