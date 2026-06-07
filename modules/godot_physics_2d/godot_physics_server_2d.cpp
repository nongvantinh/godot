/**************************************************************************/
/*  godot_physics_server_2d.cpp                                           */
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

#include "godot_physics_server_2d.h"

#include "godot_body_direct_state_2d.h"
#include "godot_broad_phase_2d_bvh.h"
#include "godot_collision_solver_2d.h"

#include "core/debugger/engine_debugger.h"
#include "core/io/marshalls.h"
#include "core/os/os.h"
#include "core/profiling/profiling.h"

#define FLUSH_QUERY_CHECK(m_object) \
	ERR_FAIL_COND_MSG(m_object->get_space() && flushing_queries, "Can't change this state while flushing queries. Use call_deferred() or set_deferred() to change monitoring state instead.");

RID GodotPhysicsServer2D::_shape_create(ShapeType p_shape) {
	GodotShape2D *shape = nullptr;
	switch (p_shape) {
		case SHAPE_WORLD_BOUNDARY: {
			shape = memnew(GodotWorldBoundaryShape2D);
		} break;
		case SHAPE_SEPARATION_RAY: {
			shape = memnew(GodotSeparationRayShape2D);
		} break;
		case SHAPE_SEGMENT: {
			shape = memnew(GodotSegmentShape2D);
		} break;
		case SHAPE_CIRCLE: {
			shape = memnew(GodotCircleShape2D);
		} break;
		case SHAPE_RECTANGLE: {
			shape = memnew(GodotRectangleShape2D);
		} break;
		case SHAPE_CAPSULE: {
			shape = memnew(GodotCapsuleShape2D);
		} break;
		case SHAPE_CONVEX_POLYGON: {
			shape = memnew(GodotConvexPolygonShape2D);
		} break;
		case SHAPE_CONCAVE_POLYGON: {
			shape = memnew(GodotConcavePolygonShape2D);
		} break;
		case SHAPE_CUSTOM: {
			ERR_FAIL_V(RID());

		} break;
	}

	RID id = shape_owner.make_rid(shape);
	shape->set_self(id);

	return id;
}

RID GodotPhysicsServer2D::world_boundary_shape_create() {
	return _shape_create(SHAPE_WORLD_BOUNDARY);
}

RID GodotPhysicsServer2D::separation_ray_shape_create() {
	return _shape_create(SHAPE_SEPARATION_RAY);
}

RID GodotPhysicsServer2D::segment_shape_create() {
	return _shape_create(SHAPE_SEGMENT);
}

RID GodotPhysicsServer2D::circle_shape_create() {
	return _shape_create(SHAPE_CIRCLE);
}

RID GodotPhysicsServer2D::rectangle_shape_create() {
	return _shape_create(SHAPE_RECTANGLE);
}

RID GodotPhysicsServer2D::capsule_shape_create() {
	return _shape_create(SHAPE_CAPSULE);
}

RID GodotPhysicsServer2D::convex_polygon_shape_create() {
	return _shape_create(SHAPE_CONVEX_POLYGON);
}

RID GodotPhysicsServer2D::concave_polygon_shape_create() {
	return _shape_create(SHAPE_CONCAVE_POLYGON);
}

void GodotPhysicsServer2D::shape_set_data(RID p_shape, const Variant &p_data) {
	GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	shape->set_data(p_data);
}

void GodotPhysicsServer2D::shape_set_custom_solver_bias(RID p_shape, real_t p_bias) {
	GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	shape->set_custom_bias(p_bias);
}

PhysicsServer2D::ShapeType GodotPhysicsServer2D::shape_get_type(RID p_shape) const {
	const GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_V(shape, SHAPE_CUSTOM);
	return shape->get_type();
}

Variant GodotPhysicsServer2D::shape_get_data(RID p_shape) const {
	const GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_V(shape, Variant());
	ERR_FAIL_COND_V(!shape->is_configured(), Variant());
	return shape->get_data();
}

real_t GodotPhysicsServer2D::shape_get_custom_solver_bias(RID p_shape) const {
	const GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_V(shape, 0);
	return shape->get_custom_bias();
}

void GodotPhysicsServer2D::_shape_col_cbk(const Vector2 &p_point_A, const Vector2 &p_point_B, void *p_userdata) {
	CollCbkData *cbk = static_cast<CollCbkData *>(p_userdata);

	if (cbk->max == 0) {
		return;
	}

	Vector2 rel_dir = (p_point_A - p_point_B);
	real_t rel_length2 = rel_dir.length_squared();
	if (cbk->valid_dir != Vector2()) {
		if (cbk->valid_depth < 10e20) {
			if (rel_length2 > cbk->valid_depth * cbk->valid_depth ||
					(rel_length2 > CMP_EPSILON && cbk->valid_dir.dot(rel_dir.normalized()) < CMP_EPSILON)) {
				cbk->invalid_by_dir++;
				return;
			}
		} else {
			if (rel_length2 > 0 && cbk->valid_dir.dot(rel_dir.normalized()) < CMP_EPSILON) {
				return;
			}
		}
	}

	if (cbk->amount == cbk->max) {
		//find least deep
		real_t min_depth = 1e20;
		int min_depth_idx = 0;
		for (int i = 0; i < cbk->amount; i++) {
			real_t d = cbk->ptr[i * 2 + 0].distance_squared_to(cbk->ptr[i * 2 + 1]);
			if (d < min_depth) {
				min_depth = d;
				min_depth_idx = i;
			}
		}

		if (rel_length2 < min_depth) {
			return;
		}
		cbk->ptr[min_depth_idx * 2 + 0] = p_point_A;
		cbk->ptr[min_depth_idx * 2 + 1] = p_point_B;
		cbk->passed++;

	} else {
		cbk->ptr[cbk->amount * 2 + 0] = p_point_A;
		cbk->ptr[cbk->amount * 2 + 1] = p_point_B;
		cbk->amount++;
		cbk->passed++;
	}
}

bool GodotPhysicsServer2D::shape_collide(RID p_shape_A, const Transform2D &p_xform_A, const Vector2 &p_motion_A, RID p_shape_B, const Transform2D &p_xform_B, const Vector2 &p_motion_B, Vector2 *r_results, int p_result_max, int &r_result_count) {
	GodotShape2D *shape_A = shape_owner.get_or_null(p_shape_A);
	ERR_FAIL_NULL_V(shape_A, false);
	GodotShape2D *shape_B = shape_owner.get_or_null(p_shape_B);
	ERR_FAIL_NULL_V(shape_B, false);

	if (p_result_max == 0) {
		return GodotCollisionSolver2D::solve(shape_A, p_xform_A, p_motion_A, shape_B, p_xform_B, p_motion_B, nullptr, nullptr);
	}

	CollCbkData cbk;
	cbk.max = p_result_max;
	cbk.amount = 0;
	cbk.passed = 0;
	cbk.ptr = r_results;

	bool res = GodotCollisionSolver2D::solve(shape_A, p_xform_A, p_motion_A, shape_B, p_xform_B, p_motion_B, _shape_col_cbk, &cbk);
	r_result_count = cbk.amount;
	return res;
}

RID GodotPhysicsServer2D::space_create() {
	GodotSpace2D *space = memnew(GodotSpace2D);
	RID id = space_owner.make_rid(space);
	space->set_self(id);
	RID area_id = area_create();
	GodotArea2D *area = area_owner.get_or_null(area_id);
	ERR_FAIL_NULL_V(area, RID());
	space->set_default_area(area);
	area->set_space(space);
	area->set_priority(-1);

	return id;
}

void GodotPhysicsServer2D::space_set_active(RID p_space, bool p_active) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);
	if (p_active) {
		active_spaces.insert(space);
	} else {
		active_spaces.erase(space);
	}
}

bool GodotPhysicsServer2D::space_is_active(RID p_space) const {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, false);

	return active_spaces.has(space);
}

void GodotPhysicsServer2D::space_set_param(RID p_space, SpaceParameter p_param, real_t p_value) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	space->set_param(p_param, p_value);
}

real_t GodotPhysicsServer2D::space_get_param(RID p_space, SpaceParameter p_param) const {
	const GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, 0);
	return space->get_param(p_param);
}

void GodotPhysicsServer2D::space_set_debug_contacts(RID p_space, int p_max_contacts) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);
	space->set_debug_contacts(p_max_contacts);
}

Vector<Vector2> GodotPhysicsServer2D::space_get_contacts(RID p_space) const {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, Vector<Vector2>());
	return space->get_debug_contacts();
}

int GodotPhysicsServer2D::space_get_contact_count(RID p_space) const {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, 0);
	return space->get_debug_contact_count();
}

PhysicsDirectSpaceState2D *GodotPhysicsServer2D::space_get_direct_state(RID p_space) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, nullptr);
	ERR_FAIL_COND_V_MSG((using_threads && !doing_sync) || space->is_locked(), nullptr, "Space state is inaccessible right now, wait for iteration or physics process notification.");

	return space->get_direct_state();
}

RID GodotPhysicsServer2D::area_create() {
	GodotArea2D *area = memnew(GodotArea2D);
	RID rid = area_owner.make_rid(area);
	area->set_self(rid);
	return rid;
}

void GodotPhysicsServer2D::area_set_space(RID p_area, RID p_space) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	GodotSpace2D *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_NULL(space);
	}

	if (area->get_space() == space) {
		return; //pointless
	}

	area->clear_constraints();
	area->set_space(space);
}

RID GodotPhysicsServer2D::area_get_space(RID p_area) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, RID());

	GodotSpace2D *space = area->get_space();
	if (!space) {
		return RID();
	}
	return space->get_self();
}

void GodotPhysicsServer2D::area_add_shape(RID p_area, RID p_shape, const Transform2D &p_transform, bool p_disabled) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	area->add_shape(shape, p_transform, p_disabled);
}

void GodotPhysicsServer2D::area_set_shape(RID p_area, int p_shape_idx, RID p_shape) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	ERR_FAIL_COND(!shape->is_configured());

	area->set_shape(p_shape_idx, shape);
}

void GodotPhysicsServer2D::area_set_shape_transform(RID p_area, int p_shape_idx, const Transform2D &p_transform) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_shape_transform(p_shape_idx, p_transform);
}

void GodotPhysicsServer2D::area_set_shape_disabled(RID p_area, int p_shape, bool p_disabled) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	ERR_FAIL_INDEX(p_shape, area->get_shape_count());
	FLUSH_QUERY_CHECK(area);

	area->set_shape_disabled(p_shape, p_disabled);
}

int GodotPhysicsServer2D::area_get_shape_count(RID p_area) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, -1);

	return area->get_shape_count();
}

RID GodotPhysicsServer2D::area_get_shape(RID p_area, int p_shape_idx) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, RID());

	GodotShape2D *shape = area->get_shape(p_shape_idx);
	ERR_FAIL_NULL_V(shape, RID());

	return shape->get_self();
}

Transform2D GodotPhysicsServer2D::area_get_shape_transform(RID p_area, int p_shape_idx) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, Transform2D());

	return area->get_shape_transform(p_shape_idx);
}

void GodotPhysicsServer2D::area_remove_shape(RID p_area, int p_shape_idx) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->remove_shape(p_shape_idx);
}

void GodotPhysicsServer2D::area_clear_shapes(RID p_area) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	while (area->get_shape_count()) {
		area->remove_shape(0);
	}
}

void GodotPhysicsServer2D::area_attach_object_instance_id(RID p_area, ObjectID p_id) {
	if (space_owner.owns(p_area)) {
		GodotSpace2D *space = space_owner.get_or_null(p_area);
		p_area = space->get_default_area()->get_self();
	}
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_instance_id(p_id);
}

ObjectID GodotPhysicsServer2D::area_get_object_instance_id(RID p_area) const {
	if (space_owner.owns(p_area)) {
		GodotSpace2D *space = space_owner.get_or_null(p_area);
		p_area = space->get_default_area()->get_self();
	}
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, ObjectID());
	return area->get_instance_id();
}

void GodotPhysicsServer2D::area_attach_canvas_instance_id(RID p_area, ObjectID p_id) {
	if (space_owner.owns(p_area)) {
		GodotSpace2D *space = space_owner.get_or_null(p_area);
		p_area = space->get_default_area()->get_self();
	}
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_canvas_instance_id(p_id);
}

ObjectID GodotPhysicsServer2D::area_get_canvas_instance_id(RID p_area) const {
	if (space_owner.owns(p_area)) {
		GodotSpace2D *space = space_owner.get_or_null(p_area);
		p_area = space->get_default_area()->get_self();
	}
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, ObjectID());
	return area->get_canvas_instance_id();
}

void GodotPhysicsServer2D::area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value) {
	if (space_owner.owns(p_area)) {
		GodotSpace2D *space = space_owner.get_or_null(p_area);
		p_area = space->get_default_area()->get_self();
	}
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_param(p_param, p_value);
}

void GodotPhysicsServer2D::area_set_transform(RID p_area, const Transform2D &p_transform) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_transform(p_transform);
}

Variant GodotPhysicsServer2D::area_get_param(RID p_area, AreaParameter p_param) const {
	if (space_owner.owns(p_area)) {
		GodotSpace2D *space = space_owner.get_or_null(p_area);
		p_area = space->get_default_area()->get_self();
	}
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, Variant());

	return area->get_param(p_param);
}

Transform2D GodotPhysicsServer2D::area_get_transform(RID p_area) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, Transform2D());

	return area->get_transform();
}

void GodotPhysicsServer2D::area_set_pickable(RID p_area, bool p_pickable) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_pickable(p_pickable);
}

void GodotPhysicsServer2D::area_set_monitorable(RID p_area, bool p_monitorable) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	FLUSH_QUERY_CHECK(area);

	area->set_monitorable(p_monitorable);
}

void GodotPhysicsServer2D::area_set_collision_layer(RID p_area, uint32_t p_layer) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_collision_layer(p_layer);
}

uint32_t GodotPhysicsServer2D::area_get_collision_layer(RID p_area) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, 0);

	return area->get_collision_layer();
}

void GodotPhysicsServer2D::area_set_collision_mask(RID p_area, uint32_t p_mask) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_collision_mask(p_mask);
}

uint32_t GodotPhysicsServer2D::area_get_collision_mask(RID p_area) const {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, 0);

	return area->get_collision_mask();
}

void GodotPhysicsServer2D::area_set_monitor_callback(RID p_area, const Callable &p_callback) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_monitor_callback(p_callback.is_valid() ? p_callback : Callable());
}

void GodotPhysicsServer2D::area_set_area_monitor_callback(RID p_area, const Callable &p_callback) {
	GodotArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_area_monitor_callback(p_callback.is_valid() ? p_callback : Callable());
}

/* BODY API */

RID GodotPhysicsServer2D::body_create() {
	GodotBody2D *body = memnew(GodotBody2D);
	RID rid = body_owner.make_rid(body);
	body->set_self(rid);
	return rid;
}

void GodotPhysicsServer2D::body_set_space(RID p_body, RID p_space) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	GodotSpace2D *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_NULL(space);
	}

	if (body->get_space() == space) {
		return; //pointless
	}

	body->clear_constraint_list();
	body->set_space(space);
}

RID GodotPhysicsServer2D::body_get_space(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, RID());

	GodotSpace2D *space = body->get_space();
	if (!space) {
		return RID();
	}
	return space->get_self();
}

void GodotPhysicsServer2D::body_set_mode(RID p_body, BodyMode p_mode) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	FLUSH_QUERY_CHECK(body);

	body->set_mode(p_mode);
}

PhysicsServer2D::BodyMode GodotPhysicsServer2D::body_get_mode(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, BODY_MODE_STATIC);

	return body->get_mode();
}

void GodotPhysicsServer2D::body_add_shape(RID p_body, RID p_shape, const Transform2D &p_transform, bool p_disabled) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	body->add_shape(shape, p_transform, p_disabled);
}

void GodotPhysicsServer2D::body_set_shape(RID p_body, int p_shape_idx, RID p_shape) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	GodotShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	ERR_FAIL_COND(!shape->is_configured());

	body->set_shape(p_shape_idx, shape);
}

void GodotPhysicsServer2D::body_set_shape_transform(RID p_body, int p_shape_idx, const Transform2D &p_transform) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_shape_transform(p_shape_idx, p_transform);
}

int GodotPhysicsServer2D::body_get_shape_count(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, -1);

	return body->get_shape_count();
}

RID GodotPhysicsServer2D::body_get_shape(RID p_body, int p_shape_idx) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, RID());

	GodotShape2D *shape = body->get_shape(p_shape_idx);
	ERR_FAIL_NULL_V(shape, RID());

	return shape->get_self();
}

Transform2D GodotPhysicsServer2D::body_get_shape_transform(RID p_body, int p_shape_idx) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, Transform2D());

	return body->get_shape_transform(p_shape_idx);
}

void GodotPhysicsServer2D::body_remove_shape(RID p_body, int p_shape_idx) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->remove_shape(p_shape_idx);
}

void GodotPhysicsServer2D::body_clear_shapes(RID p_body) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	while (body->get_shape_count()) {
		body->remove_shape(0);
	}
}

void GodotPhysicsServer2D::body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	ERR_FAIL_INDEX(p_shape_idx, body->get_shape_count());
	FLUSH_QUERY_CHECK(body);

	body->set_shape_disabled(p_shape_idx, p_disabled);
}

void GodotPhysicsServer2D::body_set_shape_as_one_way_collision(RID p_body, int p_shape_idx, bool p_enable, real_t p_margin, const Vector2 &p_direction) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	ERR_FAIL_INDEX(p_shape_idx, body->get_shape_count());
	FLUSH_QUERY_CHECK(body);

	body->set_shape_as_one_way_collision(p_shape_idx, p_enable, p_margin, p_direction);
}

void GodotPhysicsServer2D::body_set_continuous_collision_detection_mode(RID p_body, CCDMode p_mode) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_continuous_collision_detection_mode(p_mode);
}

GodotPhysicsServer2D::CCDMode GodotPhysicsServer2D::body_get_continuous_collision_detection_mode(RID p_body) const {
	const GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, CCD_MODE_DISABLED);

	return body->get_continuous_collision_detection_mode();
}

void GodotPhysicsServer2D::body_attach_object_instance_id(RID p_body, ObjectID p_id) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_instance_id(p_id);
}

ObjectID GodotPhysicsServer2D::body_get_object_instance_id(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, ObjectID());

	return body->get_instance_id();
}

void GodotPhysicsServer2D::body_attach_canvas_instance_id(RID p_body, ObjectID p_id) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_canvas_instance_id(p_id);
}

ObjectID GodotPhysicsServer2D::body_get_canvas_instance_id(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, ObjectID());

	return body->get_canvas_instance_id();
}

void GodotPhysicsServer2D::body_set_collision_layer(RID p_body, uint32_t p_layer) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_collision_layer(p_layer);
}

uint32_t GodotPhysicsServer2D::body_get_collision_layer(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);

	return body->get_collision_layer();
}

void GodotPhysicsServer2D::body_set_collision_mask(RID p_body, uint32_t p_mask) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_collision_mask(p_mask);
}

uint32_t GodotPhysicsServer2D::body_get_collision_mask(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);

	return body->get_collision_mask();
}

void GodotPhysicsServer2D::body_set_collision_priority(RID p_body, real_t p_priority) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_collision_priority(p_priority);
}

real_t GodotPhysicsServer2D::body_get_collision_priority(RID p_body) const {
	const GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);

	return body->get_collision_priority();
}

void GodotPhysicsServer2D::body_set_param(RID p_body, BodyParameter p_param, const Variant &p_value) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_param(p_param, p_value);
}

Variant GodotPhysicsServer2D::body_get_param(RID p_body, BodyParameter p_param) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);

	return body->get_param(p_param);
}

void GodotPhysicsServer2D::body_reset_mass_properties(RID p_body) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->reset_mass_properties();
}

void GodotPhysicsServer2D::body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_state(p_state, p_variant);
}

Variant GodotPhysicsServer2D::body_get_state(RID p_body, BodyState p_state) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, Variant());

	return body->get_state(p_state);
}

void GodotPhysicsServer2D::body_apply_central_impulse(RID p_body, const Vector2 &p_impulse) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->apply_central_impulse(p_impulse);
	body->wakeup();
}

void GodotPhysicsServer2D::body_apply_torque_impulse(RID p_body, real_t p_torque) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	_update_shapes();

	body->apply_torque_impulse(p_torque);
	body->wakeup();
}

void GodotPhysicsServer2D::body_apply_impulse(RID p_body, const Vector2 &p_impulse, const Vector2 &p_position) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	_update_shapes();

	body->apply_impulse(p_impulse, p_position);
	body->wakeup();
}

void GodotPhysicsServer2D::body_apply_central_force(RID p_body, const Vector2 &p_force) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->apply_central_force(p_force);
	body->wakeup();
}

void GodotPhysicsServer2D::body_apply_force(RID p_body, const Vector2 &p_force, const Vector2 &p_position) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->apply_force(p_force, p_position);
	body->wakeup();
}

void GodotPhysicsServer2D::body_apply_torque(RID p_body, real_t p_torque) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->apply_torque(p_torque);
	body->wakeup();
}

void GodotPhysicsServer2D::body_add_constant_central_force(RID p_body, const Vector2 &p_force) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_constant_central_force(p_force);
	body->wakeup();
}

void GodotPhysicsServer2D::body_add_constant_force(RID p_body, const Vector2 &p_force, const Vector2 &p_position) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_constant_force(p_force, p_position);
	body->wakeup();
}

void GodotPhysicsServer2D::body_add_constant_torque(RID p_body, real_t p_torque) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_constant_torque(p_torque);
	body->wakeup();
}

void GodotPhysicsServer2D::body_set_constant_force(RID p_body, const Vector2 &p_force) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_constant_force(p_force);
	if (!p_force.is_zero_approx()) {
		body->wakeup();
	}
}

Vector2 GodotPhysicsServer2D::body_get_constant_force(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_constant_force();
}

void GodotPhysicsServer2D::body_set_constant_torque(RID p_body, real_t p_torque) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_constant_torque(p_torque);
	if (!Math::is_zero_approx(p_torque)) {
		body->wakeup();
	}
}

real_t GodotPhysicsServer2D::body_get_constant_torque(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);

	return body->get_constant_torque();
}

void GodotPhysicsServer2D::body_set_axis_velocity(RID p_body, const Vector2 &p_axis_velocity) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	_update_shapes();

	Vector2 v = body->get_linear_velocity();
	Vector2 axis = p_axis_velocity.normalized();
	v -= axis * axis.dot(v);
	v += p_axis_velocity;
	body->set_linear_velocity(v);
	body->wakeup();
}

void GodotPhysicsServer2D::body_add_collision_exception(RID p_body, RID p_body_b) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_exception(p_body_b);
	body->wakeup();
}

void GodotPhysicsServer2D::body_remove_collision_exception(RID p_body, RID p_body_b) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->remove_exception(p_body_b);
	body->wakeup();
}

void GodotPhysicsServer2D::body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	for (int i = 0; i < body->get_exceptions().size(); i++) {
		p_exceptions->push_back(body->get_exceptions()[i]);
	}
}

void GodotPhysicsServer2D::body_set_contacts_reported_depth_threshold(RID p_body, real_t p_threshold) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
}

real_t GodotPhysicsServer2D::body_get_contacts_reported_depth_threshold(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);
	return 0;
}

void GodotPhysicsServer2D::body_set_omit_force_integration(RID p_body, bool p_omit) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_omit_force_integration(p_omit);
}

bool GodotPhysicsServer2D::body_is_omitting_force_integration(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, false);
	return body->get_omit_force_integration();
}

void GodotPhysicsServer2D::body_set_max_contacts_reported(RID p_body, int p_contacts) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_max_contacts_reported(p_contacts);
}

int GodotPhysicsServer2D::body_get_max_contacts_reported(RID p_body) const {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, -1);
	return body->get_max_contacts_reported();
}

void GodotPhysicsServer2D::body_set_state_sync_callback(RID p_body, const Callable &p_callable) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_state_sync_callback(p_callable);
}

void GodotPhysicsServer2D::body_set_force_integration_callback(RID p_body, const Callable &p_callable, const Variant &p_udata) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_force_integration_callback(p_callable, p_udata);
}

bool GodotPhysicsServer2D::body_collide_shape(RID p_body, int p_body_shape, RID p_shape, const Transform2D &p_shape_xform, const Vector2 &p_motion, Vector2 *r_results, int p_result_max, int &r_result_count) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, false);
	ERR_FAIL_INDEX_V(p_body_shape, body->get_shape_count(), false);

	return shape_collide(body->get_shape(p_body_shape)->get_self(), body->get_transform() * body->get_shape_transform(p_body_shape), Vector2(), p_shape, p_shape_xform, p_motion, r_results, p_result_max, r_result_count);
}

void GodotPhysicsServer2D::body_set_pickable(RID p_body, bool p_pickable) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_pickable(p_pickable);
}

bool GodotPhysicsServer2D::body_test_motion(RID p_body, const MotionParameters &p_parameters, MotionResult *r_result) {
	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, false);
	ERR_FAIL_NULL_V(body->get_space(), false);
	ERR_FAIL_COND_V(body->get_space()->is_locked(), false);

	_update_shapes();

	return body->get_space()->test_body_motion(body, p_parameters, r_result);
}

PhysicsDirectBodyState2D *GodotPhysicsServer2D::body_get_direct_state(RID p_body) {
	ERR_FAIL_COND_V_MSG((using_threads && !doing_sync), nullptr, "Body state is inaccessible right now, wait for iteration or physics process notification.");

	if (!body_owner.owns(p_body)) {
		return nullptr;
	}

	GodotBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, nullptr);

	if (!body->get_space()) {
		return nullptr;
	}

	ERR_FAIL_COND_V_MSG(body->get_space()->is_locked(), nullptr, "Body state is inaccessible right now, wait for iteration or physics process notification.");

	return body->get_direct_state();
}

/* JOINT API */

RID GodotPhysicsServer2D::joint_create() {
	GodotJoint2D *joint = memnew(GodotJoint2D);
	RID joint_rid = joint_owner.make_rid(joint);
	joint->set_self(joint_rid);
	return joint_rid;
}

void GodotPhysicsServer2D::joint_clear(RID p_joint) {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);
	if (joint->get_type() != JOINT_TYPE_MAX) {
		GodotJoint2D *empty_joint = memnew(GodotJoint2D);
		empty_joint->copy_settings_from(joint);

		joint_owner.replace(p_joint, empty_joint);
		memdelete(joint);
	}
}

void GodotPhysicsServer2D::joint_set_param(RID p_joint, JointParam p_param, real_t p_value) {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	switch (p_param) {
		case JOINT_PARAM_BIAS:
			joint->set_bias(p_value);
			break;
		case JOINT_PARAM_MAX_BIAS:
			joint->set_max_bias(p_value);
			break;
		case JOINT_PARAM_MAX_FORCE:
			joint->set_max_force(p_value);
			break;
	}
}

real_t GodotPhysicsServer2D::joint_get_param(RID p_joint, JointParam p_param) const {
	const GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, -1);

	switch (p_param) {
		case JOINT_PARAM_BIAS:
			return joint->get_bias();
			break;
		case JOINT_PARAM_MAX_BIAS:
			return joint->get_max_bias();
			break;
		case JOINT_PARAM_MAX_FORCE:
			return joint->get_max_force();
			break;
	}

	return 0;
}

void GodotPhysicsServer2D::joint_disable_collisions_between_bodies(RID p_joint, const bool p_disable) {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	joint->disable_collisions_between_bodies(p_disable);

	if (2 == joint->get_body_count()) {
		GodotBody2D *body_a = *joint->get_body_ptr();
		GodotBody2D *body_b = *(joint->get_body_ptr() + 1);

		if (p_disable) {
			body_add_collision_exception(body_a->get_self(), body_b->get_self());
			body_add_collision_exception(body_b->get_self(), body_a->get_self());
		} else {
			body_remove_collision_exception(body_a->get_self(), body_b->get_self());
			body_remove_collision_exception(body_b->get_self(), body_a->get_self());
		}
	}
}

bool GodotPhysicsServer2D::joint_is_disabled_collisions_between_bodies(RID p_joint) const {
	const GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, true);

	return joint->is_disabled_collisions_between_bodies();
}

void GodotPhysicsServer2D::joint_make_pin(RID p_joint, const Vector2 &p_pos, RID p_body_a, RID p_body_b) {
	GodotBody2D *A = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(A);
	GodotBody2D *B = nullptr;
	if (body_owner.owns(p_body_b)) {
		B = body_owner.get_or_null(p_body_b);
		ERR_FAIL_NULL(B);
	}

	GodotJoint2D *prev_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(prev_joint);

	GodotJoint2D *joint = memnew(GodotPinJoint2D(p_pos, A, B));

	joint_owner.replace(p_joint, joint);
	joint->copy_settings_from(prev_joint);
	memdelete(prev_joint);
}

void GodotPhysicsServer2D::joint_make_groove(RID p_joint, const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, RID p_body_a, RID p_body_b) {
	GodotBody2D *A = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(A);

	GodotBody2D *B = body_owner.get_or_null(p_body_b);
	ERR_FAIL_NULL(B);

	GodotJoint2D *prev_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(prev_joint);

	GodotJoint2D *joint = memnew(GodotGrooveJoint2D(p_a_groove1, p_a_groove2, p_b_anchor, A, B));

	joint_owner.replace(p_joint, joint);
	joint->copy_settings_from(prev_joint);
	memdelete(prev_joint);
}

void GodotPhysicsServer2D::joint_make_damped_spring(RID p_joint, const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, RID p_body_a, RID p_body_b) {
	GodotBody2D *A = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(A);

	GodotBody2D *B = body_owner.get_or_null(p_body_b);
	ERR_FAIL_NULL(B);

	GodotJoint2D *prev_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(prev_joint);

	GodotJoint2D *joint = memnew(GodotDampedSpringJoint2D(p_anchor_a, p_anchor_b, A, B));

	joint_owner.replace(p_joint, joint);
	joint->copy_settings_from(prev_joint);
	memdelete(prev_joint);
}

void GodotPhysicsServer2D::pin_joint_set_flag(RID p_joint, PinJointFlag p_flag, bool p_enabled) {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_PIN);

	GodotPinJoint2D *pin_joint = static_cast<GodotPinJoint2D *>(joint);
	pin_joint->set_flag(p_flag, p_enabled);
}

bool GodotPhysicsServer2D::pin_joint_get_flag(RID p_joint, PinJointFlag p_flag) const {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, false);
	ERR_FAIL_COND_V(joint->get_type() != JOINT_TYPE_PIN, false);

	GodotPinJoint2D *pin_joint = static_cast<GodotPinJoint2D *>(joint);
	return pin_joint->get_flag(p_flag);
}

void GodotPhysicsServer2D::pin_joint_set_param(RID p_joint, PinJointParam p_param, real_t p_value) {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_PIN);

	GodotPinJoint2D *pin_joint = static_cast<GodotPinJoint2D *>(joint);
	pin_joint->set_param(p_param, p_value);
}

real_t GodotPhysicsServer2D::pin_joint_get_param(RID p_joint, PinJointParam p_param) const {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, 0);
	ERR_FAIL_COND_V(joint->get_type() != JOINT_TYPE_PIN, 0);

	GodotPinJoint2D *pin_joint = static_cast<GodotPinJoint2D *>(joint);
	return pin_joint->get_param(p_param);
}

void GodotPhysicsServer2D::damped_spring_joint_set_param(RID p_joint, DampedSpringParam p_param, real_t p_value) {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_DAMPED_SPRING);

	GodotDampedSpringJoint2D *dsj = static_cast<GodotDampedSpringJoint2D *>(joint);
	dsj->set_param(p_param, p_value);
}

real_t GodotPhysicsServer2D::damped_spring_joint_get_param(RID p_joint, DampedSpringParam p_param) const {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, 0);
	ERR_FAIL_COND_V(joint->get_type() != JOINT_TYPE_DAMPED_SPRING, 0);

	GodotDampedSpringJoint2D *dsj = static_cast<GodotDampedSpringJoint2D *>(joint);
	return dsj->get_param(p_param);
}

PhysicsServer2D::JointType GodotPhysicsServer2D::joint_get_type(RID p_joint) const {
	GodotJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, JOINT_TYPE_PIN);

	return joint->get_type();
}

void GodotPhysicsServer2D::free_rid(RID p_rid) {
	_update_shapes(); // just in case

	if (shape_owner.owns(p_rid)) {
		GodotShape2D *shape = shape_owner.get_or_null(p_rid);

		while (shape->get_owners().size()) {
			GodotShapeOwner2D *so = shape->get_owners().begin()->key;
			so->remove_shape(shape);
		}

		shape_owner.free(p_rid);
		memdelete(shape);
	} else if (body_owner.owns(p_rid)) {
		GodotBody2D *body = body_owner.get_or_null(p_rid);

		body_set_space(p_rid, RID());

		while (body->get_shape_count()) {
			body->remove_shape(0);
		}

		body_owner.free(p_rid);
		memdelete(body);

	} else if (area_owner.owns(p_rid)) {
		GodotArea2D *area = area_owner.get_or_null(p_rid);

		area->set_space(nullptr);

		while (area->get_shape_count()) {
			area->remove_shape(0);
		}

		area_owner.free(p_rid);
		memdelete(area);
	} else if (space_owner.owns(p_rid)) {
		GodotSpace2D *space = space_owner.get_or_null(p_rid);

		while (space->get_objects().size()) {
			GodotCollisionObject2D *co = static_cast<GodotCollisionObject2D *>(*space->get_objects().begin());
			co->set_space(nullptr);
		}

		active_spaces.erase(space);
		free_rid(space->get_default_area()->get_self());
		space_owner.free(p_rid);
		memdelete(space);
	} else if (joint_owner.owns(p_rid)) {
		GodotJoint2D *joint = joint_owner.get_or_null(p_rid);

		joint_owner.free(p_rid);
		memdelete(joint);

	} else {
		ERR_FAIL_MSG("Invalid ID.");
	}
}

void GodotPhysicsServer2D::set_active(bool p_active) {
	active = p_active;
}

void GodotPhysicsServer2D::init() {
	doing_sync = false;
	stepper = memnew(GodotStep2D);
}

void GodotPhysicsServer2D::step(real_t p_step) {
	if (!active) {
		return;
	}

	_update_shapes();

	island_count = 0;
	active_objects = 0;
	collision_pairs = 0;
	for (GodotSpace2D *E : active_spaces) {
		stepper->step(E, p_step);
		island_count += E->get_island_count();
		active_objects += E->get_active_objects();
		collision_pairs += E->get_collision_pairs();
	}
}

void GodotPhysicsServer2D::space_step(RID p_space, real_t p_delta) {
	GodotProfileZone("Physics Step (manual 2D)");
	if (!active) {
		return;
	}

	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	ERR_FAIL_COND_MSG(!Math::is_finite(p_delta) || p_delta < 0, "space_step: delta must be finite and non-negative.");

	_update_shapes();

	stepper->step(space, p_delta);
}

void GodotPhysicsServer2D::space_flush_queries(RID p_space) {
	GodotProfileZone("Physics Flush Queries (manual 2D)");
	if (!active) {
		return;
	}

	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	flushing_queries = true;
	space->call_queries();
	flushing_queries = false;
}

// ---------------------------------------------------------------------------
// State snapshot helpers — GodotPhysics 2D (PARTIAL fidelity)
// ---------------------------------------------------------------------------
//
// See the companion 3D implementation for a full design description.
// Blob format: GPSS header (dim=2, backend=0) + BODIES section + AREAS section.

static const uint8_t GPSS_MAGIC_GP2[4] = { 'G', 'P', 'S', 'S' };
static const uint8_t GPSS_DIM_2D = 2;
static const uint8_t GPSS_VERSION_GP2 = 1;
static const uint8_t GPSS_BACKEND_GODOT2 = 0;
static const uint16_t GPSS_SEC_BODIES_GP2 = 2;
static const uint16_t GPSS_SEC_AREAS_GP2 = 3;
static const int GPSS_HDR_SIZE2 = 12;
static const int GPSS_SEC_HDR_SIZE2 = 6;

// Per-body record 2D: rid_id(8) + transform2d(24) + linear_vel(8) + angular_vel(4) + active(1) = 45 bytes
static const int GP2D_BODY_RECORD_SIZE = 45;
// Per-area record 2D: rid_id(8) + transform2d(24) = 32 bytes
static const int GP2D_AREA_RECORD_SIZE = 32;

static void _write_float2(uint8_t *dst, float v) {
	uint32_t bits;
	memcpy(&bits, &v, 4);
	encode_uint32(bits, dst);
}
static float _read_float2(const uint8_t *src) {
	uint32_t bits = decode_uint32(src);
	float v;
	memcpy(&v, &bits, 4);
	return v;
}

static void _write_vec2(uint8_t *dst, const Vector2 &v) {
	_write_float2(dst + 0, (float)v.x);
	_write_float2(dst + 4, (float)v.y);
}
static Vector2 _read_vec2(const uint8_t *src) {
	return Vector2(_read_float2(src + 0), _read_float2(src + 4));
}

static void _write_transform2d(uint8_t *dst, const Transform2D &t) {
	_write_vec2(dst + 0, t.columns[0]);
	_write_vec2(dst + 8, t.columns[1]);
	_write_vec2(dst + 16, t.columns[2]);
}
static Transform2D _read_transform2d(const uint8_t *src) {
	Transform2D t;
	t.columns[0] = _read_vec2(src + 0);
	t.columns[1] = _read_vec2(src + 8);
	t.columns[2] = _read_vec2(src + 16);
	return t;
}

PackedByteArray GodotPhysicsServer2D::space_save_state(RID p_space) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V_MSG(space, PackedByteArray(), "space_save_state: invalid space RID.");

	LocalVector<GodotBody2D *> bodies;
	LocalVector<GodotArea2D *> areas;

	for (GodotCollisionObject2D *obj : space->get_objects()) {
		if (obj->get_type() == GodotCollisionObject2D::TYPE_BODY) {
			bodies.push_back(static_cast<GodotBody2D *>(obj));
		} else if (obj->get_type() == GodotCollisionObject2D::TYPE_AREA) {
			areas.push_back(static_cast<GodotArea2D *>(obj));
		}
	}

	uint32_t body_count = (uint32_t)bodies.size();
	uint32_t area_count = (uint32_t)areas.size();

	uint32_t bodies_payload_size = 4 + body_count * (uint32_t)GP2D_BODY_RECORD_SIZE;
	uint32_t areas_payload_size = 4 + area_count * (uint32_t)GP2D_AREA_RECORD_SIZE;
	uint32_t section_count = 2;

	int total_size = GPSS_HDR_SIZE2 + GPSS_SEC_HDR_SIZE2 + (int)bodies_payload_size + GPSS_SEC_HDR_SIZE2 + (int)areas_payload_size;

	PackedByteArray blob;
	blob.resize(total_size);
	uint8_t *w = blob.ptrw();

	memcpy(w, GPSS_MAGIC_GP2, 4);
	w[4] = GPSS_DIM_2D;
	w[5] = GPSS_VERSION_GP2;
	w[6] = GPSS_BACKEND_GODOT2;
	w[7] = 0;
	encode_uint32(section_count, w + 8);

	uint8_t *sp = w + GPSS_HDR_SIZE2;
	encode_uint16(GPSS_SEC_BODIES_GP2, sp);
	encode_uint32(bodies_payload_size, sp + 2);
	sp += GPSS_SEC_HDR_SIZE2;

	encode_uint32(body_count, sp);
	sp += 4;
	for (GodotBody2D *body : bodies) {
		encode_uint64(body->get_self().get_id(), sp);
		_write_transform2d(sp + 8, body->get_transform());
		_write_vec2(sp + 32, body->get_linear_velocity());
		_write_float2(sp + 40, (float)body->get_angular_velocity());
		sp[44] = body->is_active() ? 1 : 0;
		sp += GP2D_BODY_RECORD_SIZE;
	}

	encode_uint16(GPSS_SEC_AREAS_GP2, sp);
	encode_uint32(areas_payload_size, sp + 2);
	sp += GPSS_SEC_HDR_SIZE2;

	encode_uint32(area_count, sp);
	sp += 4;
	for (GodotArea2D *area : areas) {
		encode_uint64(area->get_self().get_id(), sp);
		_write_transform2d(sp + 8, area->get_transform());
		sp += GP2D_AREA_RECORD_SIZE;
	}

	return blob;
}

bool GodotPhysicsServer2D::space_restore_state(RID p_space, const PackedByteArray &p_state) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V_MSG(space, false, "space_restore_state: invalid space RID.");

	const uint8_t *r = p_state.ptr();
	int64_t size = p_state.size();

	if (size < GPSS_HDR_SIZE2) {
		WARN_PRINT("space_restore_state: blob too small for header.");
		return false;
	}
	if (memcmp(r, GPSS_MAGIC_GP2, 4) != 0) {
		WARN_PRINT("space_restore_state: wrong magic bytes.");
		return false;
	}
	if (r[4] != GPSS_DIM_2D) {
		WARN_PRINT("space_restore_state: dimension mismatch (blob is not a 2D space snapshot).");
		return false;
	}
	if (r[5] != GPSS_VERSION_GP2) {
		WARN_PRINT("space_restore_state: unsupported format version.");
		return false;
	}
	if (r[6] != GPSS_BACKEND_GODOT2) {
		WARN_PRINT("space_restore_state: backend id mismatch (blob was not written by the GodotPhysics backend).");
		return false;
	}
	if (r[7] != 0) {
		WARN_PRINT("space_restore_state: reserved byte is non-zero.");
		return false;
	}

	uint32_t section_count = decode_uint32(r + 8);
	int64_t pos = GPSS_HDR_SIZE2;

	struct BodyRecord2D {
		uint64_t rid_id;
		Transform2D transform;
		Vector2 linear_velocity;
		real_t angular_velocity;
		bool active;
	};
	struct AreaRecord2D {
		uint64_t rid_id;
		Transform2D transform;
	};

	LocalVector<BodyRecord2D> staged_bodies;
	LocalVector<AreaRecord2D> staged_areas;
	bool bodies_parsed = false;
	bool areas_parsed = false;

	for (uint32_t i = 0; i < section_count; i++) {
		if (pos + GPSS_SEC_HDR_SIZE2 > size) {
			WARN_PRINT("space_restore_state: section header truncated.");
			return false;
		}
		uint16_t tag = decode_uint16(r + pos);
		uint32_t sec_len = decode_uint32(r + pos + 2);
		pos += GPSS_SEC_HDR_SIZE2;

		if ((int64_t)sec_len > size - pos) {
			WARN_PRINT("space_restore_state: section length exceeds remaining buffer.");
			return false;
		}

		if (tag == GPSS_SEC_BODIES_GP2) {
			const uint8_t *sp = r + pos;
			const uint8_t *sp_end = sp + sec_len;
			if (sec_len < 4) {
				WARN_PRINT("space_restore_state: BODIES section too small.");
				return false;
			}
			uint32_t body_count = decode_uint32(sp);
			sp += 4;
			if ((uint64_t)body_count * GP2D_BODY_RECORD_SIZE > (uint64_t)(sp_end - sp)) {
				WARN_PRINT("space_restore_state: body count overflows section payload.");
				return false;
			}
			staged_bodies.resize(body_count);
			for (uint32_t bi = 0; bi < body_count; bi++) {
				staged_bodies[bi].rid_id = decode_uint64(sp);
				staged_bodies[bi].transform = _read_transform2d(sp + 8);
				staged_bodies[bi].linear_velocity = _read_vec2(sp + 32);
				staged_bodies[bi].angular_velocity = (real_t)_read_float2(sp + 40);
				staged_bodies[bi].active = (sp[44] != 0);
				sp += GP2D_BODY_RECORD_SIZE;
			}
			bodies_parsed = true;
		} else if (tag == GPSS_SEC_AREAS_GP2) {
			const uint8_t *sp = r + pos;
			const uint8_t *sp_end = sp + sec_len;
			if (sec_len < 4) {
				WARN_PRINT("space_restore_state: AREAS section too small.");
				return false;
			}
			uint32_t area_count = decode_uint32(sp);
			sp += 4;
			if ((uint64_t)area_count * GP2D_AREA_RECORD_SIZE > (uint64_t)(sp_end - sp)) {
				WARN_PRINT("space_restore_state: area count overflows section payload.");
				return false;
			}
			staged_areas.resize(area_count);
			for (uint32_t ai = 0; ai < area_count; ai++) {
				staged_areas[ai].rid_id = decode_uint64(sp);
				staged_areas[ai].transform = _read_transform2d(sp + 8);
				sp += GP2D_AREA_RECORD_SIZE;
			}
			areas_parsed = true;
		}
		pos += (int64_t)sec_len;
	}

	if (pos != size) {
		WARN_PRINT("space_restore_state: trailing garbage after sections.");
		return false;
	}
	if (!bodies_parsed) {
		WARN_PRINT("space_restore_state: no BODIES section found in blob.");
		return false;
	}

	space->reset_debug_contact_count();

	for (const BodyRecord2D &rec : staged_bodies) {
		RID rid = RID::from_uint64(rec.rid_id);
		GodotBody2D *body = body_owner.get_or_null(rid);
		if (!body || body->get_space() != space) {
			continue;
		}
		body->set_state(BODY_STATE_TRANSFORM, rec.transform);
		body->set_linear_velocity(rec.linear_velocity);
		body->set_angular_velocity(rec.angular_velocity);
		body->set_active(rec.active);
	}

	if (areas_parsed) {
		for (const AreaRecord2D &rec : staged_areas) {
			RID rid = RID::from_uint64(rec.rid_id);
			GodotArea2D *area = area_owner.get_or_null(rid);
			if (!area || area->get_space() != space) {
				continue;
			}
			area->set_transform(rec.transform);
		}
	}

	return true;
}

void GodotPhysicsServer2D::space_reset(RID p_space) {
	GodotSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_MSG(space, "space_reset: invalid space RID.");

	GodotArea2D *default_area = space->get_default_area();

	LocalVector<GodotCollisionObject2D *> objs;
	for (GodotCollisionObject2D *obj : space->get_objects()) {
		if (obj == static_cast<GodotCollisionObject2D *>(default_area)) {
			continue;
		}
		objs.push_back(obj);
	}
	for (GodotCollisionObject2D *obj : objs) {
		obj->set_space(nullptr);
	}
}

bool GodotPhysicsServer2D::space_clone_state(RID p_src_space, RID p_dst_space) {
	GodotSpace2D *src = space_owner.get_or_null(p_src_space);
	ERR_FAIL_NULL_V_MSG(src, false, "space_clone_state: invalid src_space RID.");
	GodotSpace2D *dst = space_owner.get_or_null(p_dst_space);
	ERR_FAIL_NULL_V_MSG(dst, false, "space_clone_state: invalid dst_space RID.");
	ERR_FAIL_COND_V_MSG(src == dst, false, "space_clone_state: src_space and dst_space must differ.");

	// Collect bodies in stable insertion order (NOT get_objects(), HashSet order is unstable).
	LocalVector<GodotBody2D *> src_bodies;
	LocalVector<GodotBody2D *> dst_bodies;

	for (GodotCollisionObject2D *obj : src->get_objects_ordered()) {
		if (obj->get_type() == GodotCollisionObject2D::TYPE_BODY) {
			src_bodies.push_back(static_cast<GodotBody2D *>(obj));
		}
	}
	for (GodotCollisionObject2D *obj : dst->get_objects_ordered()) {
		if (obj->get_type() == GodotCollisionObject2D::TYPE_BODY) {
			dst_bodies.push_back(static_cast<GodotBody2D *>(obj));
		}
	}

	// All-or-nothing: body count must match.
	if (src_bodies.size() != dst_bodies.size()) {
		WARN_PRINT("space_clone_state: body count mismatch between src and dst spaces, no state was written.");
		return false;
	}

	// Stage all source state before writing any destination body (no partial write).
	struct BodyState2D {
		Transform2D transform;
		Vector2 linear_velocity;
		real_t angular_velocity;
		bool active;
	};
	LocalVector<BodyState2D> staged;
	staged.resize(src_bodies.size());
	for (uint32_t i = 0; i < src_bodies.size(); i++) {
		staged[i].transform = src_bodies[i]->get_transform();
		staged[i].linear_velocity = src_bodies[i]->get_linear_velocity();
		staged[i].angular_velocity = src_bodies[i]->get_angular_velocity();
		staged[i].active = src_bodies[i]->is_active();
	}

	// Apply staged state to dst bodies pairwise (ordinal i -> ordinal i).
	for (uint32_t i = 0; i < dst_bodies.size(); i++) {
		dst_bodies[i]->set_state(BODY_STATE_TRANSFORM, staged[i].transform);
		dst_bodies[i]->set_linear_velocity(staged[i].linear_velocity);
		dst_bodies[i]->set_angular_velocity(staged[i].angular_velocity);
		dst_bodies[i]->set_active(staged[i].active);
	}

	return true;
}

int GodotPhysicsServer2D::space_get_feature(RID p_space, SpaceFeature p_feature) const {
	if (p_feature == FEATURE_STATE_SNAPSHOT) {
		return SPACE_FEATURE_PARTIAL;
	}
	if (p_feature == FEATURE_STATE_CLONE) {
		return SPACE_FEATURE_PARTIAL;
	}
	if (p_feature == FEATURE_MANUAL_STEP) {
		return SPACE_FEATURE_FULL;
	}
	return SPACE_FEATURE_NONE;
}

void GodotPhysicsServer2D::sync() {
	doing_sync = true;
}

void GodotPhysicsServer2D::flush_queries() {
	if (!active) {
		return;
	}

	flushing_queries = true;

	uint64_t time_beg = OS::get_singleton()->get_ticks_usec();

	for (GodotSpace2D *E : active_spaces) {
		E->call_queries();
	}

	flushing_queries = false;

	if (EngineDebugger::is_profiling("servers")) {
		uint64_t total_time[GodotSpace2D::ELAPSED_TIME_MAX];
		static const char *time_name[GodotSpace2D::ELAPSED_TIME_MAX] = {
			"integrate_forces",
			"generate_islands",
			"setup_constraints",
			"solve_constraints",
			"integrate_velocities"
		};

		for (int i = 0; i < GodotSpace2D::ELAPSED_TIME_MAX; i++) {
			total_time[i] = 0;
		}

		for (const GodotSpace2D *E : active_spaces) {
			for (int i = 0; i < GodotSpace2D::ELAPSED_TIME_MAX; i++) {
				total_time[i] += E->get_elapsed_time(GodotSpace2D::ElapsedTime(i));
			}
		}

		Array values;
		values.resize(GodotSpace2D::ELAPSED_TIME_MAX * 2);
		for (int i = 0; i < GodotSpace2D::ELAPSED_TIME_MAX; i++) {
			values[i * 2 + 0] = time_name[i];
			values[i * 2 + 1] = USEC_TO_SEC(total_time[i]);
		}
		values.push_back("flush_queries");
		values.push_back(USEC_TO_SEC(OS::get_singleton()->get_ticks_usec() - time_beg));

		values.push_front("physics_2d");
		EngineDebugger::profiler_add_frame_data("servers", values);
	}
}

void GodotPhysicsServer2D::end_sync() {
	doing_sync = false;
}

void GodotPhysicsServer2D::finish() {
	memdelete(stepper);
}

void GodotPhysicsServer2D::_update_shapes() {
	while (pending_shape_update_list.first()) {
		pending_shape_update_list.first()->self()->_shape_changed();
		pending_shape_update_list.remove(pending_shape_update_list.first());
	}
}

int GodotPhysicsServer2D::get_process_info(ProcessInfo p_info) {
	switch (p_info) {
		case INFO_ACTIVE_OBJECTS: {
			return active_objects;
		} break;
		case INFO_COLLISION_PAIRS: {
			return collision_pairs;
		} break;
		case INFO_ISLAND_COUNT: {
			return island_count;
		} break;
	}

	return 0;
}

GodotPhysicsServer2D *GodotPhysicsServer2D::godot_singleton = nullptr;

GodotPhysicsServer2D::GodotPhysicsServer2D(bool p_using_threads) {
	godot_singleton = this;
	GodotBroadPhase2D::create_func = GodotBroadPhase2DBVH::_create;

	using_threads = p_using_threads;
}
