/**************************************************************************/
/*  test_physics_server_2d_space_state.cpp                                */
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

TEST_FORCE_LINK(test_physics_server_2d_space_state)

#ifndef PHYSICS_2D_DISABLED

#include "core/object/worker_thread_pool.h"
#include "servers/physics_2d/physics_server_2d.h"
#include "servers/physics_2d/physics_server_2d_dummy.h"

namespace TestPhysicsServer2DSpaceState {

static bool is_dummy_server() {
	return Object::cast_to<PhysicsServer2DDummy>(PhysicsServer2D::get_singleton()) != nullptr;
}

static real_t get_body_y(PhysicsServer2D *ps, RID body) {
	Variant v = ps->body_get_state(body, PhysicsServer2D::BODY_STATE_TRANSFORM);
	return ((Transform2D)v).get_origin().y;
}

TEST_SUITE("[PhysicsServer2D][SpaceState]") {
	// -----------------------------------------------------------------------
	// space_get_feature
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_get_feature returns NONE for dummy server") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		if (!is_dummy_server()) {
			WARN("Test targets dummy server but a real backend is loaded — skipping.");
			return;
		}
		RID space = ps->space_create();
		CHECK_EQ(ps->space_get_feature(space, PhysicsServer2D::FEATURE_STATE_SNAPSHOT),
				(int)PhysicsServer2D::SPACE_FEATURE_NONE);
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_get_feature returns at-least-PARTIAL for real backend") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		ps->space_set_active(space, true);
		int feat = ps->space_get_feature(space, PhysicsServer2D::FEATURE_STATE_SNAPSHOT);
		CHECK_MESSAGE(feat >= (int)PhysicsServer2D::SPACE_FEATURE_PARTIAL,
				"Real 2D backend must report at least PARTIAL for FEATURE_STATE_SNAPSHOT.");
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// space_save_state
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_save_state returns non-empty blob for live space") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		RID space = ps->space_create();
		ps->space_set_active(space, true);
		PackedByteArray blob = ps->space_save_state(space);
		CHECK_MESSAGE(blob.size() > 0, "space_save_state must return a non-empty blob.");
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_save_state returns empty for invalid RID") {
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);
		ERR_PRINT_OFF;
		PackedByteArray blob = ps->space_save_state(RID());
		ERR_PRINT_ON;
		CHECK_EQ(blob.size(), 0);
	}

	// -----------------------------------------------------------------------
	// Round-trip: save → mutate → restore
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state round-trip restores body state") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 0.5f);

		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PhysicsServer2D::BODY_MODE_RIGID);
		const Vector2 start_pos(0, 200);
		ps->body_set_state(body, PhysicsServer2D::BODY_STATE_TRANSFORM,
				Transform2D(0, start_pos));

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE_MESSAGE(blob.size() > 0, "save_state must produce a non-empty blob.");

		// Mutate: step so body moves.
		ps->space_step_safe(space, 1.0f / 60.0f);
		real_t y_after_step = get_body_y(ps, body);
		CHECK_MESSAGE(y_after_step != start_pos.y,
				"Body should have moved after a step.");

		// Restore.
		bool ok = ps->space_restore_state(space, blob);
		CHECK_MESSAGE(ok, "space_restore_state must return true for valid blob.");

		real_t y_restored = get_body_y(ps, body);
		CHECK_MESSAGE(Math::is_equal_approx(y_restored, start_pos.y, (real_t)0.01),
				"Body Y must be back at save-point after restore.");

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// space_reset
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_reset detaches all bodies without freeing RIDs") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 0.5f);
		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PhysicsServer2D::BODY_MODE_RIGID);

		CHECK_EQ(ps->body_get_space(body), space);

		ps->space_reset(space);

		RID body_space = ps->body_get_space(body);
		CHECK_MESSAGE(!body_space.is_valid(),
				"After space_reset body's space must be RID() (detached).");

		// Re-add: handle must still be valid.
		ps->body_set_space(body, space);
		CHECK_EQ(ps->body_get_space(body), space);

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// B1 regression: space_reset must NOT detach the default area — the space
	// must remain valid-and-usable after reset.
	// This test FAILS against the iter-1 buggy reset (which detached everything).
	TEST_CASE("[SceneTree][PhysicsServer2D] space_reset preserves infrastructure: space is functional after reset") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		REQUIRE(ps != nullptr);

		RID space = ps->space_create();
		ps->space_set_active(space, true);

		// Add a user body, then reset.
		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 0.5f);
		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PhysicsServer2D::BODY_MODE_RIGID);
		const Vector2 start_pos(0, 200);
		ps->body_set_state(body, PhysicsServer2D::BODY_STATE_TRANSFORM,
				Transform2D(0, start_pos));

		ps->space_reset(space);

		// User body must be detached.
		CHECK_MESSAGE(!ps->body_get_space(body).is_valid(),
				"User body must be detached after space_reset.");

		// The space must still be valid: save_state must return a non-empty blob
		// (which requires the internal header machinery and default area to be intact).
		PackedByteArray blob_after_reset = ps->space_save_state(space);
		CHECK_MESSAGE(blob_after_reset.size() > 0,
				"space_save_state must succeed after space_reset — default area must still be attached.");

		// Re-add the body to the reset space and step once to confirm functionality.
		ps->body_set_space(body, space);
		CHECK_EQ(ps->body_get_space(body), space);

		ps->space_step_safe(space, 1.0f / 60.0f);
		real_t y_after = get_body_y(ps, body);
		CHECK_MESSAGE(y_after != start_pos.y,
				"Body must move after being re-added to a reset space — space is functional.");

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// Malformed-blob rejection
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects empty blob") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, PackedByteArray());
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects wrong magic") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() >= 4);
		blob.ptrw()[0] = 'X';

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects dimension mismatch (3D blob into 2D)") {
		// This tests that a 3D blob is rejected by the 2D server.
		// We manually craft a minimal blob with dim=3 and valid magic/version/backend.
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() >= 5);
		// Flip the dimension byte from 2 to 3.
		blob.ptrw()[4] = 3;

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects section length exceeding buffer") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() >= 18);
		uint8_t *w = blob.ptrw();
		// Corrupt first section length.
		w[14] = 0xFF;
		w[15] = 0xFF;
		w[16] = 0xFF;
		w[17] = 0xFF;

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC#8: blob rejection — wrong format version (byte [5])
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects wrong version") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() >= 6);
		blob.ptrw()[5] = 0xFF; // Corrupt format-version byte.

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC#8: blob rejection — non-zero reserved byte (byte [7])
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects non-zero reserved byte") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() >= 8);
		blob.ptrw()[7] = 0x01; // Corrupt reserved byte.

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC#8: blob rejection — truncated blob (mid-section payload)
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects truncated blob") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		// Save with a body present so the blob has non-trivial content.
		RID shape = ps->circle_shape_create();
		ps->shape_set_data(shape, 0.5f);
		RID body = ps->body_create();
		ps->body_set_space(body, space);
		ps->body_add_shape(body, shape);
		ps->body_set_mode(body, PhysicsServer2D::BODY_MODE_RIGID);

		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() > 20);

		// Truncate by half.
		blob.resize(blob.size() / 2);

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);

		ps->free_rid(body);
		ps->free_rid(shape);
		ps->free_rid(space);
	}

	// -----------------------------------------------------------------------
	// AC#8: blob rejection — backend-id mismatch (byte [6])
	// -----------------------------------------------------------------------

	TEST_CASE("[SceneTree][PhysicsServer2D] space_restore_state rejects backend id mismatch") {
		if (is_dummy_server()) {
			WARN("Skipping: dummy server.");
			return;
		}
		PhysicsServer2D *ps = PhysicsServer2D::get_singleton();
		RID space = ps->space_create();
		ps->space_set_active(space, true);

		// Get a valid 2D blob, then flip backend-id byte to an unknown value.
		PackedByteArray blob = ps->space_save_state(space);
		REQUIRE(blob.size() >= 7);
		blob.ptrw()[6] = 0x7F; // Unknown backend id (neither 0=GodotPhysics nor 1=Jolt).

		ERR_PRINT_OFF;
		bool ok = ps->space_restore_state(space, blob);
		ERR_PRINT_ON;
		CHECK_FALSE(ok);
		ps->free_rid(space);
	}

} // TEST_SUITE

} // namespace TestPhysicsServer2DSpaceState

#endif // PHYSICS_2D_DISABLED
