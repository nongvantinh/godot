/**************************************************************************/
/*  test_physics_server_2d_space_clone.cpp                                */
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

// 2D parity doctest for space_clone_state.
//
// Mirrors test_physics_server_3d_space_clone.cpp: every test case has a 2D
// counterpart so the AC coverage table is symmetric across dimensions.
//
// Covered ACs:
//   AC1: ordinal-copy: matching body counts, state copied by registration order.
//   AC3: count-mismatch: returns false, no partial write (all-or-nothing).
//   AC3: src == dst guard: returns false.
//   AC3: invalid-RID guards: both src-invalid and dst-invalid return false.
//   AC4: FEATURE_STATE_CLONE: dummy returns NONE; real 2D backend returns PARTIAL.
//   AC2: two-budget trajectory: clone + N steps tracks src within tolerances.

#include "tests/test_macros.h"

TEST_FORCE_LINK(test_physics_server_2d_space_clone)

#ifndef PHYSICS_2D_DISABLED

#include "servers/physics_2d/physics_server_2d.h"
#include "servers/physics_2d/physics_server_2d_dummy.h"

namespace TestPhysicsServer2DSpaceClone {

static bool is_dummy_server() {
	return Object::cast_to<PhysicsServer2DDummy>(PhysicsServer2D::get_singleton()) != nullptr;
}

static Vector2 get_body_origin(PhysicsServer2D *ps, RID body) {
	Variant v = ps->body_get_state(body, PhysicsServer2D::BODY_STATE_TRANSFORM);
	return ((Transform2D)v).get_origin();
}

// Helper: create a circle body in a 2D space at the given position.
// r_shape receives the created shape RID so the caller can free it.
static RID make_body(PhysicsServer2D *ps, RID space, const Vector2 &p_pos, RID &r_shape) {
	r_shape = ps->circle_shape_create();
	ps->shape_set_data(r_shape, 0.5f);
	RID body = ps->body_create();
	ps->body_set_space(body, space);
	ps->body_add_shape(body, r_shape);
	ps->body_set_mode(body, PhysicsServer2D::BODY_MODE_RIGID);
	ps->body_set_state(body, PhysicsServer2D::BODY_STATE_TRANSFORM,
			Transform2D(0.0f, p_pos));
	return body;
}

TEST_SUITE("[PhysicsServer2D][SpaceClone]") {
	// -----------------------------------------------------------------------
	// FEATURE_STATE_CLONE enum value and space_get_feature reports.
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] FEATURE_STATE_CLONE: dummy returns NONE") {
		if (!is_dummy_server()) {
			MESSAGE("Test targets dummy server but a real backend is loaded, skipping.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		CHECK_EQ(ps->space_get_feature(space, PhysicsServer2D::FEATURE_STATE_CLONE),
				(int)PhysicsServer2D::SPACE_FEATURE_NONE);
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] FEATURE_STATE_CLONE: real backend returns PARTIAL") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		ps->space_set_active(space, true);
		int feat = ps->space_get_feature(space, PhysicsServer2D::FEATURE_STATE_CLONE);
		CHECK_MESSAGE(feat == (int)PhysicsServer2D::SPACE_FEATURE_PARTIAL,
				"Real 2D backend must report PARTIAL for FEATURE_STATE_CLONE (field-set copy only, no warm-start).");
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC1: space_clone_state: round-trip ordinal copy.
	// Stepping dst after clone must NOT perturb src (isolation).
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: copies body state by ordinal") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID src_space = ps->space_create();
		ps->space_set_active(src_space, true);
		RID dst_space = ps->space_create();
		ps->space_set_active(dst_space, true);

		// Create matching bodies in src and dst (same registration order).
		RID src_shape, dst_shape;
		const Vector2 src_pos(1.0f, 10.0f);
		RID src_body = make_body(ps, src_space, src_pos, src_shape);
		const Vector2 dst_pos(0.0f, 0.0f);
		RID dst_body = make_body(ps, dst_space, dst_pos, dst_shape);

		// Set a known linear velocity on src so we can verify it was cloned.
		const Vector2 src_vel(3.0f, 0.0f);
		ps->body_set_state(src_body, PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY, src_vel);

		bool ok = ps->space_clone_state(src_space, dst_space);
		CHECK_MESSAGE(ok, "space_clone_state must return true for matching body counts.");

		// dst body must now have src body's position.
		Vector2 dst_origin = get_body_origin(ps, dst_body);
		CHECK_MESSAGE(Math::is_equal_approx(dst_origin.x, src_pos.x, (real_t)0.01),
				"Clone: dst body X must match src body X.");
		CHECK_MESSAGE(Math::is_equal_approx(dst_origin.y, src_pos.y, (real_t)0.01),
				"Clone: dst body Y must match src body Y.");

		// dst body must now have src body's linear velocity.
		Variant lv_v = ps->body_get_state(dst_body, PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY);
		Vector2 dst_vel = (Vector2)lv_v;
		CHECK_MESSAGE(Math::is_equal_approx(dst_vel.x, src_vel.x, (real_t)0.01),
				"Clone: dst body linear velocity X must match src.");

		// Isolation: step dst only — src must remain unchanged.
		Vector2 src_origin_before = get_body_origin(ps, src_body);
		ps->space_step_safe(dst_space, 1.0f / 60.0f);
		Vector2 src_origin_after = get_body_origin(ps, src_body);
		CHECK_MESSAGE(Math::is_equal_approx(src_origin_after.x, src_origin_before.x, (real_t)0.01),
				"Isolation: stepping dst must not change src body X.");
		CHECK_MESSAGE(Math::is_equal_approx(src_origin_after.y, src_origin_before.y, (real_t)0.01),
				"Isolation: stepping dst must not change src body Y.");

		ps->free_rid(src_body);
		ps->free_rid(dst_body);
		ps->free_rid(src_shape);
		ps->free_rid(dst_shape);
		ps->free_rid(src_space);
		ps->free_rid(dst_space);
	}

	// -----------------------------------------------------------------------
	// AC3: space_clone_state: count mismatch returns false, no partial write.
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: count mismatch returns false, no partial write") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID src_space = ps->space_create();
		ps->space_set_active(src_space, true);
		RID dst_space = ps->space_create();
		ps->space_set_active(dst_space, true);

		// src has one body, dst has two, mismatch.
		RID src_shape;
		const Vector2 src_pos(7.0f, 7.0f);
		RID src_body = make_body(ps, src_space, src_pos, src_shape);

		const Vector2 dst_pos_before(0.0f, 0.0f);
		RID dst_shape_a, dst_shape_b;
		RID dst_body_a = make_body(ps, dst_space, dst_pos_before, dst_shape_a);
		RID dst_body_b = make_body(ps, dst_space, Vector2(1.0f, 1.0f), dst_shape_b);

		ERR_PRINT_OFF;
		bool ok = ps->space_clone_state(src_space, dst_space);
		ERR_PRINT_ON;
		CHECK_MESSAGE(!ok, "space_clone_state must return false on body count mismatch.");

		// dst_body_a must still be at its original position (no partial write).
		Vector2 dst_origin = get_body_origin(ps, dst_body_a);
		CHECK_MESSAGE(Math::is_equal_approx(dst_origin.x, dst_pos_before.x, (real_t)0.01),
				"On count mismatch dst body must be unchanged (no partial write) — X.");
		CHECK_MESSAGE(Math::is_equal_approx(dst_origin.y, dst_pos_before.y, (real_t)0.01),
				"On count mismatch dst body must be unchanged (no partial write) — Y.");

		ps->free_rid(src_body);
		ps->free_rid(dst_body_a);
		ps->free_rid(dst_body_b);
		ps->free_rid(src_shape);
		ps->free_rid(dst_shape_a);
		ps->free_rid(dst_shape_b);
		ps->free_rid(src_space);
		ps->free_rid(dst_space);
	}

	// -----------------------------------------------------------------------
	// AC3: space_clone_state: src == dst returns false.
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: src == dst returns false") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		ps->space_set_active(space, true);
		ERR_PRINT_OFF;
		bool ok = ps->space_clone_state(space, space);
		ERR_PRINT_ON;
		CHECK_MESSAGE(!ok, "space_clone_state(space, space) must return false.");
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC3: space_clone_state: invalid-RID guards.
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: invalid src RID returns false") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		ERR_PRINT_OFF;
		bool ok = ps->space_clone_state(RID(), space);
		ERR_PRINT_ON;
		CHECK_MESSAGE(!ok, "space_clone_state with invalid src must return false.");
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: invalid dst RID returns false") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		ERR_PRINT_OFF;
		bool ok = ps->space_clone_state(space, RID());
		ERR_PRINT_ON;
		CHECK_MESSAGE(!ok, "space_clone_state with invalid dst must return false.");
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC2: Two-budget trajectory tolerance (2D parity).
	//
	// After clone + N steps, dst trajectory matches a continuous src run within:
	//   first-step budget  : < 0.5 px (cold-contact transient)
	//   steady-state budget: < 1e-3 px per step (propose-and-measure)
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: cloned space trajectory within two-budget tolerance") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID src_space = ps->space_create();
		ps->space_set_active(src_space, true);
		RID dst_space = ps->space_create();
		ps->space_set_active(dst_space, true);

		const real_t dt = (real_t)(1.0 / 60.0);
		// Start high so the body is in free-fall for N steps without hitting anything.
		const Vector2 start(0.0f, -200.0f);

		RID src_shape, dst_shape;
		RID src_body = make_body(ps, src_space, start, src_shape);
		RID dst_body = make_body(ps, dst_space, start, dst_shape);

		// Step src once to settle from cold start, then clone to dst.
		ps->space_step_safe(src_space, dt);

		bool ok = ps->space_clone_state(src_space, dst_space);
		REQUIRE_MESSAGE(ok, "space_clone_state must succeed for matching body counts.");

		// Step both src and dst N times and compare positions step-by-step.
		const int N = 5;
		for (int i = 0; i < N; i++) {
			ps->space_step_safe(src_space, dt);
			ps->space_step_safe(dst_space, dt);

			Vector2 src_origin = get_body_origin(ps, src_body);
			Vector2 dst_origin = get_body_origin(ps, dst_body);

			real_t diff = (dst_origin - src_origin).length();
			if (i == 0) {
				// First-step budget: cold-contact transient allowed up to 0.5 px.
				CHECK_MESSAGE(diff < (real_t)0.5,
						"Clone first-step budget (2D): dst vs src position must be within 0.5 px.");
			} else {
				// Steady-state per-step budget: < 1e-3 px per step (propose-and-measure).
				CHECK_MESSAGE(diff < (real_t)1e-3,
						"Clone steady-state budget (2D): dst vs src position must be within 1e-3 px per step.");
			}
		}

		ps->free_rid(src_body);
		ps->free_rid(dst_body);
		ps->free_rid(src_shape);
		ps->free_rid(dst_shape);
		ps->free_rid(src_space);
		ps->free_rid(dst_space);
	}

	// -----------------------------------------------------------------------
	// 2D parallel: space_clone_state pairs by insertion order, not
	// by RID id. dst bodies are freed and recreated in reverse order so their
	// RID ids diverge from src; insertion-order pairing (GodotSpace2D
	// objects_ordered) must still copy to the correct ordinal.
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: pairs by insertion order under RID-id reordering") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID src_space = ps->space_create();
		ps->space_set_active(src_space, true);
		RID dst_space = ps->space_create();
		ps->space_set_active(dst_space, true);

		RID src_shape_a, src_shape_b;
		const Vector2 pos_a(10, 20);
		const Vector2 pos_b(30, 40);
		RID src_a = make_body(ps, src_space, pos_a, src_shape_a);
		RID src_b = make_body(ps, src_space, pos_b, src_shape_b);

		RID dst_shape_a, dst_shape_b;
		RID dst_a = make_body(ps, dst_space, Vector2(0, 0), dst_shape_a);
		RID dst_b = make_body(ps, dst_space, Vector2(0, 0), dst_shape_b);

		// Free + recreate dst bodies in reverse order so RID ids diverge.
		ps->free_rid(dst_a);
		ps->free_rid(dst_shape_a);
		ps->free_rid(dst_b);
		ps->free_rid(dst_shape_b);

		RID dst_shape_0, dst_shape_1;
		RID dst_ord0 = make_body(ps, dst_space, Vector2(0, 0), dst_shape_0); // ordinal 0
		RID dst_ord1 = make_body(ps, dst_space, Vector2(0, 0), dst_shape_1); // ordinal 1

		bool ok = ps->space_clone_state(src_space, dst_space);
		REQUIRE_MESSAGE(ok, "2D space_clone_state must succeed for matching body counts after dst reorder.");

		Vector2 o0 = get_body_origin(ps, dst_ord0);
		Vector2 o1 = get_body_origin(ps, dst_ord1);
		CHECK_MESSAGE(o0.distance_to(pos_a) < (real_t)0.01,
				"Clone after reorder (2D): dst ordinal-0 must receive src body A by insertion order.");
		CHECK_MESSAGE(o1.distance_to(pos_b) < (real_t)0.01,
				"Clone after reorder (2D): dst ordinal-1 must receive src body B by insertion order.");

		ps->free_rid(src_a);
		ps->free_rid(src_b);
		ps->free_rid(dst_ord0);
		ps->free_rid(dst_ord1);
		ps->free_rid(src_shape_a);
		ps->free_rid(src_shape_b);
		ps->free_rid(dst_shape_0);
		ps->free_rid(dst_shape_1);
		ps->free_rid(src_space);
		ps->free_rid(dst_space);
	}

	// -----------------------------------------------------------------------
	// dummy space_clone_state returns false with no write. The
	// WARN_PRINT is not asserted (warnings are not captured); the no-write/
	// false contract is. Runs only when the dummy server is active.
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_clone_state: dummy returns false") {
		if (!is_dummy_server()) {
			MESSAGE("Test targets dummy server but a real backend is loaded, skipping.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID src = ps->space_create();
		RID dst = ps->space_create();
		ERR_PRINT_OFF;
		bool ok = ps->space_clone_state(src, dst);
		ERR_PRINT_ON;
		CHECK_MESSAGE(!ok, "Dummy space_clone_state must return false (FEATURE_STATE_CLONE is NONE).");
		ps->free_rid(src);
		ps->free_rid(dst);
	}

} // TEST_SUITE

} // namespace TestPhysicsServer2DSpaceClone

#endif // PHYSICS_2D_DISABLED
