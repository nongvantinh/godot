/**************************************************************************/
/*  jolt_stream_wrappers.h                                                */
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

#include "core/variant/variant.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Physics/StateRecorder.h>

// In-memory StateRecorder backed by a PackedByteArray.
//
// JoltMemoryStateRecorderOut: used for save_state, writes into an owned PackedByteArray.
// JoltMemoryStateRecorderIn:  used for restore_state, reads from a borrowed PackedByteArray.
// Both subclass JPH::StateRecorder (which itself inherits StreamIn + StreamOut).
// Only the relevant stream direction (Read vs Write) is ever called by Jolt
// when the recorder is used for save or restore respectively.

class JoltMemoryStateRecorderOut final : public JPH::StateRecorder {
	PackedByteArray &data;
	bool failed = false;

public:
	explicit JoltMemoryStateRecorderOut(PackedByteArray &p_data) :
			data(p_data) {}

	virtual void WriteBytes(const void *p_data_ptr, size_t p_bytes) override {
		if (failed || p_bytes == 0) {
			return;
		}
		int64_t old_size = data.size();
		int64_t new_size = old_size + (int64_t)p_bytes;
		data.resize(new_size);
		if (data.size() != new_size) {
			failed = true;
			return;
		}
		memcpy(data.ptrw() + old_size, p_data_ptr, p_bytes);
	}

	virtual bool IsFailed() const override { return failed; }

	// ReadBytes not used during save; stub that marks failure if called unexpectedly.
	virtual void ReadBytes(void *, size_t) override { failed = true; }
	virtual bool IsEOF() const override { return true; }
};

class JoltMemoryStateRecorderIn final : public JPH::StateRecorder {
	const PackedByteArray &data;
	int64_t cursor = 0;
	bool failed = false;

public:
	explicit JoltMemoryStateRecorderIn(const PackedByteArray &p_data) :
			data(p_data) {}

	virtual void ReadBytes(void *p_data_ptr, size_t p_bytes) override {
		if (failed || p_bytes == 0) {
			return;
		}
		if (cursor + (int64_t)p_bytes > data.size()) {
			failed = true;
			return;
		}
		memcpy(p_data_ptr, data.ptr() + cursor, p_bytes);
		cursor += (int64_t)p_bytes;
	}

	virtual bool IsEOF() const override { return cursor >= data.size(); }
	virtual bool IsFailed() const override { return failed; }

	// WriteBytes not used during restore; stub.
	virtual void WriteBytes(const void *, size_t) override { failed = true; }
};

#ifdef DEBUG_ENABLED

#include "core/io/file_access.h"

class JoltStreamOutputWrapper final : public JPH::StreamOut {
	Ref<FileAccess> file_access;

public:
	explicit JoltStreamOutputWrapper(const Ref<FileAccess> &p_file_access) :
			file_access(p_file_access) {}

	virtual void WriteBytes(const void *p_data, size_t p_bytes) override {
		file_access->store_buffer(static_cast<const uint8_t *>(p_data), static_cast<uint64_t>(p_bytes));
	}

	virtual bool IsFailed() const override {
		return file_access->get_error() != OK;
	}
};

class JoltStreamInputWrapper final : public JPH::StreamIn {
	Ref<FileAccess> file_access;

public:
	explicit JoltStreamInputWrapper(const Ref<FileAccess> &p_file_access) :
			file_access(p_file_access) {}

	virtual void ReadBytes(void *p_data, size_t p_bytes) override {
		file_access->get_buffer(static_cast<uint8_t *>(p_data), static_cast<uint64_t>(p_bytes));
	}

	virtual bool IsEOF() const override {
		return file_access->eof_reached();
	}

	virtual bool IsFailed() const override {
		return file_access->get_error() != OK;
	}
};

#endif
