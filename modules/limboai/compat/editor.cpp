/**************************************************************************/
/*  editor.cpp                                                            */
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

/**
 * editor.cpp
 * =============================================================================
 * Copyright (c) 2023-present Serhii Snitsaruk and the LimboAI contributors.
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 * =============================================================================
 */
#include "editor.h"

#ifdef TOOLS_ENABLED

#ifdef LIMBOAI_MODULE
#include "editor/editor_main_screen.h"
#include "editor/editor_node.h"
#include "editor/plugins/script_editor_plugin.h"
#endif // LIMBOAI_MODULE

#ifdef LIMBOAI_GDEXTENSION
#include "resource_loader.h"
#include <godot_cpp/classes/script.hpp>
#include <godot_cpp/classes/script_editor.hpp>
#include <godot_cpp/classes/script_editor_base.hpp>
#endif // LIMBOAI_GDEXTENSION

void SHOW_BUILTIN_DOC(const String &p_topic) {
#ifdef LIMBOAI_MODULE
	ScriptEditor::get_singleton()->goto_help(p_topic);
	EditorNode::get_singleton()->get_editor_main_screen()->select(EditorMainScreen::EDITOR_SCRIPT);
#elif LIMBOAI_GDEXTENSION
	TypedArray<ScriptEditorBase> open_editors = EditorInterface::get_singleton()->get_script_editor()->get_open_script_editors();
	ERR_FAIL_COND_MSG(open_editors.size() == 0, "Can't open help page. Need at least one script open in the script editor.");
	ScriptEditorBase *seb = Object::cast_to<ScriptEditorBase>(open_editors.front());
	ERR_FAIL_NULL(seb);
	seb->emit_signal("go_to_help", p_topic);
#endif
}

void EDIT_SCRIPT(const String &p_path) {
#ifdef LIMBOAI_MODULE
	Ref<Resource> res = ScriptEditor::get_singleton()->open_file(p_path);
	ERR_FAIL_COND_MSG(res.is_null(), "Failed to load script: " + p_path);
	EditorNode::get_singleton()->edit_resource(res);
#elif LIMBOAI_GDEXTENSION
	Ref<Script> res = RESOURCE_LOAD(p_path, "Script");
	ERR_FAIL_COND_MSG(res.is_null(), "Failed to load script: " + p_path);
	EditorInterface::get_singleton()->edit_script(res);
	EditorInterface::get_singleton()->set_main_screen_editor("Script");
#endif
}

#endif // ! TOOLS_ENABLED
