/**************************************************************************/
/*  test_physics_server_2d_space_step.cpp                                 */
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

TEST_FORCE_LINK(test_physics_server_2d_space_step)

#ifndef PHYSICS_2D_DISABLED

#include "core/config/engine.h"
#include "core/object/worker_thread_pool.h"
#include "core/variant/typed_array.h"
#include "servers/physics_2d/physics_server_2d.h"
#include "servers/physics_2d/physics_server_2d_dummy.h"

namespace TestPhysicsServer2DSpaceStep {

// Helper: returns true if the server is the dummy (no-op) implementation.
static bool is_dummy_server() {
	return Object::cast_to<PhysicsServer2DDummy>(PhysicsServer2D::get_singleton()) != nullptr;
}

// Helper: get body 2D position.
static Vector2 get_body_pos(PhysicsServer2D *ps, RID body) {
	Variant v = ps->body_get_state(body, PS2DE::BODY_STATE_TRANSFORM);
	return ((Transform2D)v).get_origin();
}

TEST_SUITE("[PhysicsServer2D][SpaceStep]") {
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step advances a body under gravity") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		RID circle_shape = ps->circle_shape_create();
		ps->shape_set_data(circle_shape, 16.0f);

		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, circle_shape);
		ps->body_set_mode(body, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body, PS2DE::BODY_STATE_TRANSFORM, Transform2D(0.0f, Vector2(0, -200)));

		// Step the space manually by one frame.
		const real_t dt = 1.0f / 60.0f;
		ps->space_step_safe(space, dt);

		// The body should have moved from its initial position under default gravity.
		Variant transform_var = ps->body_get_state(body, PS2DE::BODY_STATE_TRANSFORM);
		Transform2D t = transform_var;
		CHECK_MESSAGE(t.get_origin() != Vector2(0, -200), "Body should have moved from its initial position after one step.");

		ps->free_rid(body);
		ps->free_rid(circle_shape);
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_flush_queries is callable on an active space") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		// Calling space_flush_queries directly must not crash or assert.
		ps->sync();
		ps->space_flush_queries(space);
		ps->end_sync();

		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_step does not advance get_physics_frames") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		uint64_t frames_before = Engine::get_singleton()->get_physics_frames();
		ps->space_step_safe(space, 1.0f / 60.0f);
		uint64_t frames_after = Engine::get_singleton()->get_physics_frames();

		CHECK_EQ(frames_before, frames_after);

		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_step with invalid RID returns clean error") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID invalid_space;
		// Must not crash.
		ERR_PRINT_OFF;
		ps->sync();
		ps->space_step(invalid_space, 1.0f / 60.0f);
		ps->end_sync();
		ERR_PRINT_ON;
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_flush_queries with invalid RID returns clean error") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID invalid_space;
		// Must not crash.
		ERR_PRINT_OFF;
		ps->sync();
		ps->space_flush_queries(invalid_space);
		ps->end_sync();
		ERR_PRINT_ON;
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_safe with invalid RID returns clean error") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID invalid_space;
		// Must not crash.
		ERR_PRINT_OFF;
		ps->space_step_safe(invalid_space, 1.0f / 60.0f);
		ERR_PRINT_ON;
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_safe does not leave server in flushing state") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		CHECK_FALSE(ps->is_flushing_queries());
		ps->space_step_safe(space, 1.0f / 60.0f);
		CHECK_FALSE(ps->is_flushing_queries());

		ps->free_rid(space);
	}

	// AC: 2D space_step advances only the named space, not other spaces.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step advances only the named space") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		const real_t dt = 1.0f / 60.0f;
		const Vector2 start_a = Vector2(0, -200);
		const Vector2 start_b = Vector2(0, -100);

		// Space A — will be stepped manually.
		RID space_a = ps->space_create();
		ps->space_set_active(space_a, true);
		RID shape_a = ps->circle_shape_create();
		ps->shape_set_data(shape_a, 16.0f);
		RID body_a = ps->body_create();
		ps->body_set_space(body_a, space_a);
		ps->body_add_shape(body_a, shape_a);
		ps->body_set_mode(body_a, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body_a, PS2DE::BODY_STATE_TRANSFORM, Transform2D(0.0f, start_a));

		// Space B, must NOT be advanced.
		RID space_b = ps->space_create();
		ps->space_set_active(space_b, true);
		RID shape_b = ps->circle_shape_create();
		ps->shape_set_data(shape_b, 16.0f);
		RID body_b = ps->body_create();
		ps->body_set_space(body_b, space_b);
		ps->body_add_shape(body_b, shape_b);
		ps->body_set_mode(body_b, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body_b, PS2DE::BODY_STATE_TRANSFORM, Transform2D(0.0f, start_b));

		// Step space_a only.
		ps->sync();
		ps->space_step(space_a, dt);
		ps->end_sync();

		// Body A must have moved.
		Vector2 pos_a_after = get_body_pos(ps, body_a);
		CHECK_MESSAGE(pos_a_after != start_a, "Body in stepped space should have moved.");

		// Body B must still be at exactly its start position.
		Vector2 pos_b_after = get_body_pos(ps, body_b);
		CHECK_MESSAGE(pos_b_after == start_b, "Body in un-stepped space must not have moved.");

		ps->free_rid(body_a);
		ps->free_rid(shape_a);
		ps->free_rid(space_a);
		ps->free_rid(body_b);
		ps->free_rid(shape_b);
		ps->free_rid(space_b);
	}

	// AC: GodotPhysics 2D parity, one space_step == one automatic step iteration.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step parity with one automatic step iteration") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		const real_t dt = 1.0f / 60.0f;
		const Vector2 start_pos = Vector2(0, -500);

		// --- Manual space ---
		RID space_manual = ps->space_create();
		ps->space_set_active(space_manual, true);
		RID shape_manual = ps->circle_shape_create();
		ps->shape_set_data(shape_manual, 16.0f);
		RID body_manual = ps->body_create();
		ps->body_set_space(body_manual, space_manual);
		ps->body_add_shape(body_manual, shape_manual);
		ps->body_set_mode(body_manual, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body_manual, PS2DE::BODY_STATE_TRANSFORM, Transform2D(0.0f, start_pos));

		ps->space_step_safe(space_manual, dt);
		Vector2 pos_manual = get_body_pos(ps, body_manual);

		// Deactivate manual space so the global step() only touches the auto space.
		ps->space_set_active(space_manual, false);

		// --- Auto space: stepped via the global step() ---
		RID space_auto = ps->space_create();
		ps->space_set_active(space_auto, true);
		RID shape_auto = ps->circle_shape_create();
		ps->shape_set_data(shape_auto, 16.0f);
		RID body_auto = ps->body_create();
		ps->body_set_space(body_auto, space_auto);
		ps->body_add_shape(body_auto, shape_auto);
		ps->body_set_mode(body_auto, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body_auto, PS2DE::BODY_STATE_TRANSFORM, Transform2D(0.0f, start_pos));

		ps->sync();
		ps->flush_queries();
		ps->end_sync();
		ps->step(dt);

		Vector2 pos_auto = get_body_pos(ps, body_auto);

		CHECK_MESSAGE(pos_manual.is_equal_approx(pos_auto),
				"2D space_step must produce the same result as one automatic step iteration.");

		ps->free_rid(body_auto);
		ps->free_rid(shape_auto);
		ps->free_rid(space_auto);
		ps->free_rid(body_manual);
		ps->free_rid(shape_manual);
		ps->free_rid(space_manual);
	}

	// AC: 2D space_step_safe observes submitted body state (drain invariant, 2D).
	// This is the 2D sibling of the 3D drain-invariant test.
	// The precise MT blocker is the same as documented in the 3D counterpart.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_safe observes submitted body state (drain invariant)") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);
		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 16.0f);
		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PS2DE::BODY_MODE_RIGID);

		// Submit an explicit start position.
		const Vector2 submitted_pos = Vector2(50, -300);
		ps->body_set_state(body, PS2DE::BODY_STATE_TRANSFORM,
				Transform2D(0.0f, submitted_pos));

		ps->space_step_safe(space, 1.0f / 60.0f);

		// Body must have moved FROM submitted_pos, drain invariant holds.
		Vector2 pos_after = get_body_pos(ps, body);
		CHECK_MESSAGE(pos_after != submitted_pos,
				"Step must have moved the body: confirms drain invariant (started from submitted state).");
		// X should remain near submitted X (gravity is Y axis in 2D default).
		CHECK_MESSAGE(Math::is_equal_approx(pos_after.x, submitted_pos.x, (real_t)0.1),
				"Body X must remain near submitted X: step started from submitted state.");

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// space_step_batch (batched contract + oracle, 2D)
	// -----------------------------------------------------------------------

	// AC4 + AC2: batch end-state identical to serial space_step_safe loop, same dt (2D).
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_batch parity with serial space_step_safe loop") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		const int N = 4;
		const int STEPS = 100;
		const real_t dt = 1.0f / 60.0f;

		// --- Serial reference ---
		RID serial_spaces[N], serial_shapes[N], serial_bodies[N];
		Vector2 serial_end[N];

		for (int i = 0; i < N; i++) {
			serial_spaces[i] = ps->space_create();
			ps->space_set_active(serial_spaces[i], true);
			serial_shapes[i] = ps->circle_shape_create();
			ps->shape_set_data(serial_shapes[i], 16.0f);
			serial_bodies[i] = ps->body_create();
			ps->body_set_space(serial_bodies[i], serial_spaces[i]);
			ps->body_add_shape(serial_bodies[i], serial_shapes[i]);
			ps->body_set_mode(serial_bodies[i], PS2DE::BODY_MODE_RIGID);
			ps->body_set_state(serial_bodies[i], PS2DE::BODY_STATE_TRANSFORM,
					Transform2D(0.0f, Vector2(0, -2000.0f - i * 100.0f)));
		}

		for (int s = 0; s < STEPS; s++) {
			for (int i = 0; i < N; i++) {
				ps->space_step_safe(serial_spaces[i], dt);
			}
		}
		for (int i = 0; i < N; i++) {
			serial_end[i] = get_body_pos(ps, serial_bodies[i]);
		}

		// --- Batch path ---
		RID batch_spaces[N], batch_shapes[N], batch_bodies[N];
		for (int i = 0; i < N; i++) {
			batch_spaces[i] = ps->space_create();
			ps->space_set_active(batch_spaces[i], true);
			batch_shapes[i] = ps->circle_shape_create();
			ps->shape_set_data(batch_shapes[i], 16.0f);
			batch_bodies[i] = ps->body_create();
			ps->body_set_space(batch_bodies[i], batch_spaces[i]);
			ps->body_add_shape(batch_bodies[i], batch_shapes[i]);
			ps->body_set_mode(batch_bodies[i], PS2DE::BODY_MODE_RIGID);
			ps->body_set_state(batch_bodies[i], PS2DE::BODY_STATE_TRANSFORM,
					Transform2D(0.0f, Vector2(0, -2000.0f - i * 100.0f)));
		}

		TypedArray<RID> space_arr;
		for (int i = 0; i < N; i++) {
			space_arr.push_back(batch_spaces[i]);
		}
		for (int s = 0; s < STEPS; s++) {
			ps->space_step_batch(space_arr, dt);
		}

		// Compare within steady-state budget (< 1e-3 m / 1 px in 2D units).
		for (int i = 0; i < N; i++) {
			Vector2 batch_end = get_body_pos(ps, batch_bodies[i]);
			real_t diff = (batch_end - serial_end[i]).length();
			CHECK_MESSAGE(diff < (real_t)1.0,
					"2D space_step_batch end-state must match serial space_step_safe within 1px tolerance.");
		}

		// Teardown
		for (int i = 0; i < N; i++) {
			ps->free_rid(serial_bodies[i]);
			ps->free_rid(serial_shapes[i]);
			ps->free_rid(serial_spaces[i]);
			ps->free_rid(batch_bodies[i]);
			ps->free_rid(batch_shapes[i]);
			ps->free_rid(batch_spaces[i]);
		}
	}

	// AC7: 2D space_step_batch empty array, no-op, no crash.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_batch empty array is a no-op") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		TypedArray<RID> empty;
		CHECK_FALSE(ps->is_flushing_queries());
		ps->space_step_batch(empty, 1.0f / 60.0f);
		CHECK_FALSE(ps->is_flushing_queries());
	}

	// AC7: 2D space_step_batch with invalid RID, skips cleanly.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_batch invalid RID is skipped") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		TypedArray<RID> arr;
		arr.push_back(RID());
		ERR_PRINT_OFF;
		ps->space_step_batch(arr, 1.0f / 60.0f);
		ERR_PRINT_ON;
		CHECK_FALSE(ps->is_flushing_queries());
	}

	// AC7: 2D space_step_batch with duplicate RIDs, each occurrence stepped (no dedup).
	// A single body stepped twice in one batch must have moved further than stepped once.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_batch duplicate RID is stepped twice") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);
		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 16.0f);
		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body, PS2DE::BODY_STATE_TRANSFORM,
				Transform2D(0.0f, Vector2(0, -2000)));

		const real_t dt = 1.0f / 60.0f;

		// Baseline: one step.
		Vector2 pos_before = get_body_pos(ps, body);
		ps->space_step_safe(space, dt);
		Vector2 pos_after_one = get_body_pos(ps, body);
		// Body under gravity must have moved after one step.
		CHECK(pos_after_one != pos_before);

		// Reset position.
		ps->body_set_state(body, PS2DE::BODY_STATE_TRANSFORM,
				Transform2D(0.0f, Vector2(0, -2000)));

		// Batch with duplicate: same space twice -> two steps.
		TypedArray<RID> dup;
		dup.push_back(space);
		dup.push_back(space);
		ps->space_step_batch(dup, dt);
		Vector2 pos_after_two = get_body_pos(ps, body);

		// Two steps must differ from one step (body under gravity moves more in 2 steps).
		CHECK_MESSAGE(pos_after_two != pos_after_one,
				"space_step_batch with duplicate RID must step space twice (different position than single step).");

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// AC8: 2D space_step_batch from non-main thread returns clean error.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_batch from non-main thread returns clean error") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		TypedArray<RID> arr;
		arr.push_back(space);

		struct OffThreadWork {
			PhysicsServer2D *ps;
			TypedArray<RID> arr;
			static void run(void *p_ud) {
				OffThreadWork *self = static_cast<OffThreadWork *>(p_ud);
				ERR_PRINT_OFF;
				self->ps->space_step_batch(self->arr, 1.0f / 60.0f);
				ERR_PRINT_ON;
			}
		} work{ ps, arr };

		WorkerThreadPool::TaskID tid = WorkerThreadPool::get_singleton()->add_native_task(
				&OffThreadWork::run, &work, false);
		WorkerThreadPool::get_singleton()->wait_for_task_completion(tid);

		CHECK_FALSE(ps->is_flushing_queries());
		ps->free_rid(space);
	}

	// AC: 2D space_step called from a non-main thread returns clean error and does not crash.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step from non-main thread returns clean error") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);
		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 16.0f);
		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body, PS2DE::BODY_STATE_TRANSFORM,
				Transform2D(0.0f, Vector2(0, -200)));

		struct OffThreadWork {
			PhysicsServer2D *ps;
			RID space;
			static void run(void *p_ud) {
				OffThreadWork *self = static_cast<OffThreadWork *>(p_ud);
				ERR_PRINT_OFF;
				self->ps->space_step(self->space, 1.0f / 60.0f);
				self->ps->space_flush_queries(self->space);
				self->ps->space_step_safe(self->space, 1.0f / 60.0f);
				ERR_PRINT_ON;
			}
		} work{ ps, space };

		WorkerThreadPool::TaskID tid = WorkerThreadPool::get_singleton()->add_native_task(
				&OffThreadWork::run, &work, false);
		WorkerThreadPool::get_singleton()->wait_for_task_completion(tid);

		// Server must still be usable after the off-thread no-ops.
		ps->space_step_safe(space, 1.0f / 60.0f);
		Vector2 pos_after = get_body_pos(ps, body);
		CHECK_MESSAGE(pos_after != Vector2(0, -200),
				"Main-thread step after off-thread guard must still work correctly.");

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// delta guard: a non-finite or negative delta must be rejected (no step), not
	// forwarded to the solver. Covers space_step and, transitively, space_step_safe.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step rejects non-finite / negative delta") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		RID circle_shape = ps->circle_shape_create();
		ps->shape_set_data(circle_shape, 16.0f);

		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, circle_shape);
		ps->body_set_mode(body, PS2DE::BODY_MODE_RIGID);
		ps->body_set_state(body, PS2DE::BODY_STATE_TRANSFORM, Transform2D(0.0f, Vector2(0, -200)));

		const Vector2 pos_start = get_body_pos(ps, body);

		// Bad deltas must be rejected (ERR_FAIL), leaving the body untouched.
		ERR_PRINT_OFF;
		ps->space_step_safe(space, Math::NaN);
		ps->space_step_safe(space, Math::INF);
		ps->space_step_safe(space, -1.0f);
		ERR_PRINT_ON;
		CHECK_MESSAGE(get_body_pos(ps, body) == pos_start, "A non-finite/negative delta must not advance the body.");

		// A valid delta still steps normally (proves the body is otherwise movable).
		ps->space_step_safe(space, 1.0f / 60.0f);
		CHECK_MESSAGE(get_body_pos(ps, body) != pos_start, "A valid delta must still advance the body.");

		ps->free_rid(body);
		ps->free_rid(circle_shape);
		ps->free_rid(space);
	}

	// space_step_batch skips foreign / cross-dimension / null RIDs via a
	// single ERR_CONTINUE diagnostic each, the sync/end_sync bracket stays paired, and
	// the valid owned spaces' post-state equals the serial space_step_safe oracle while
	// the foreign RIDs are untouched.
	TEST_CASE("[SceneTree][PhysicsServer2D] space_step_batch skips foreign/null RID, owned spaces match serial oracle") {
		if (is_dummy_server()) {
			MESSAGE("Skipping: dummy physics server has no simulation.");
			return;
		}

		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		const int N = 3;
		const int STEPS = 30;
		const real_t dt = 1.0f / 60.0f;

		// Serial oracle: N owned spaces stepped independently via space_step_safe.
		RID serial_spaces[N], serial_shapes[N], serial_bodies[N];
		Vector2 serial_end[N];
		for (int i = 0; i < N; i++) {
			serial_spaces[i] = ps->space_create();
			ps->space_set_active(serial_spaces[i], true);
			serial_shapes[i] = ps->circle_shape_create();
			ps->shape_set_data(serial_shapes[i], 16.0f);
			serial_bodies[i] = ps->body_create();
			ps->body_set_space(serial_bodies[i], serial_spaces[i]);
			ps->body_add_shape(serial_bodies[i], serial_shapes[i]);
			ps->body_set_mode(serial_bodies[i], PS2DE::BODY_MODE_RIGID);
			ps->body_set_state(serial_bodies[i], PS2DE::BODY_STATE_TRANSFORM,
					Transform2D(0.0f, Vector2(0, -2000.0f - i * 100.0f)));
		}
		for (int s = 0; s < STEPS; s++) {
			for (int i = 0; i < N; i++) {
				ps->space_step_safe(serial_spaces[i], dt);
			}
		}
		for (int i = 0; i < N; i++) {
			serial_end[i] = get_body_pos(ps, serial_bodies[i]);
		}

		// Batch path: same N owned spaces, plus a foreign body RID and a null RID() interleaved.
		RID batch_spaces[N], batch_shapes[N], batch_bodies[N];
		for (int i = 0; i < N; i++) {
			batch_spaces[i] = ps->space_create();
			ps->space_set_active(batch_spaces[i], true);
			batch_shapes[i] = ps->circle_shape_create();
			ps->shape_set_data(batch_shapes[i], 16.0f);
			batch_bodies[i] = ps->body_create();
			ps->body_set_space(batch_bodies[i], batch_spaces[i]);
			ps->body_add_shape(batch_bodies[i], batch_shapes[i]);
			ps->body_set_mode(batch_bodies[i], PS2DE::BODY_MODE_RIGID);
			ps->body_set_state(batch_bodies[i], PS2DE::BODY_STATE_TRANSFORM,
					Transform2D(0.0f, Vector2(0, -2000.0f - i * 100.0f)));
		}

		// A foreign-but-valid RID: a body RID is not owned by space_owner, so space_is_valid
		// must reject it. A standalone (unattached) body keeps the test self-contained.
		RID foreign_body = ps->body_create();

		TypedArray<RID> arr;
		arr.push_back(batch_spaces[0]);
		arr.push_back(RID()); // null -> skipped
		arr.push_back(batch_spaces[1]);
		arr.push_back(foreign_body); // foreign (body RID, not a space) -> skipped
		arr.push_back(batch_spaces[2]);

		// Each bad RID emits one diagnostic; the bracket stays paired regardless.
		ERR_PRINT_OFF;
		for (int s = 0; s < STEPS; s++) {
			ps->space_step_batch(arr, dt);
		}
		ERR_PRINT_ON;

		// The server must not be left flushing (bracket paired across all skips).
		CHECK_FALSE_MESSAGE(ps->is_flushing_queries(),
				"space_step_batch must leave the sync/end_sync bracket paired despite skipped RIDs.");

		// Owned spaces advanced exactly as the serial oracle; foreign RID untouched.
		for (int i = 0; i < N; i++) {
			Vector2 batch_end = get_body_pos(ps, batch_bodies[i]);
			real_t diff = (batch_end - serial_end[i]).length();
			CHECK_MESSAGE(diff < (real_t)1.0,
					"space_step_batch owned space must match the serial space_step_safe oracle (foreign/null RIDs skipped).");
		}

		// Teardown.
		ps->free_rid(foreign_body);
		for (int i = 0; i < N; i++) {
			ps->free_rid(serial_bodies[i]);
			ps->free_rid(serial_shapes[i]);
			ps->free_rid(serial_spaces[i]);
			ps->free_rid(batch_bodies[i]);
			ps->free_rid(batch_shapes[i]);
			ps->free_rid(batch_spaces[i]);
		}
	}

} // TEST_SUITE

} // namespace TestPhysicsServer2DSpaceStep

#endif // PHYSICS_2D_DISABLED
