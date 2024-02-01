#!/usr/bin/env python3
import os
import re
import subprocess
import platform
import shutil

def detect_platform():
    current_platform = platform.system().lower()
    
    if current_platform == "windows":
        return "windows"
    elif current_platform == "linux" or current_platform.startswith("bsd"):
        return "linuxbsd"
    else:
        return "macos"

PLATFORM = detect_platform()
GODOT_PROJECT_PATH = os.getcwd()
GODOT_BIN = os.path.join(GODOT_PROJECT_PATH, "bin", "godot.{0}.editor.dev.x86_64.mono.exe".format(PLATFORM))
BUILD_ASSEMBLIES_SCRIPT = os.path.join(GODOT_PROJECT_PATH, "modules", "mono", "build_scripts", "build_assemblies.py")
GODOT_OUTPUT_DIR = os.path.join(GODOT_PROJECT_PATH, "bin")
LOCAL_NUGET_SOURCE_PATH = os.path.join(GODOT_OUTPUT_DIR, "MyLocalNugetSource")
SOURCE_NAME = "MyLocalNugetSource"

# Compile with .NET
subprocess.run(["scons", "dev_build=yes", "dev_mode=yes", "vsproj=yes", "module_mono_enabled=yes"])

# Build .NET assemblies
subprocess.run(["python", BUILD_ASSEMBLIES_SCRIPT, "--godot-output-dir={0}".format(GODOT_OUTPUT_DIR), "--godot-platform={0}".format(PLATFORM), "--dev-debug"])

# Generate glue sources
subprocess.run([GODOT_BIN, "--headless", "--generate-mono-glue", os.path.join("modules","mono","glue")])

# Re-build .NET assemblies again
subprocess.run(["python", BUILD_ASSEMBLIES_SCRIPT, "--godot-output-dir={0}".format(GODOT_OUTPUT_DIR), "--godot-platform={0}".format(PLATFORM), "--dev-debug"])

# Set up NuGet source
nuget_list_output = subprocess.run(["dotnet", "nuget", "list", "source"], capture_output=True, text=True).stdout

# Make sure NuGet source is clean
if SOURCE_NAME in nuget_list_output:
    print("{0} has already exist, remove the old one.".format(SOURCE_NAME))
    subprocess.run(["dotnet", "nuget", "remove", "source", SOURCE_NAME])

# Add NuGet source paths
subprocess.run(["dotnet", "nuget", "add", "source", LOCAL_NUGET_SOURCE_PATH, "--name", SOURCE_NAME])

# Generate SOURCE_NAME
subprocess.run(["python", BUILD_ASSEMBLIES_SCRIPT, "--godot-output-dir", GODOT_OUTPUT_DIR, "--push-nupkgs-local",LOCAL_NUGET_SOURCE_PATH])

# Build export templates
subprocess.run(["scons", "target=template_debug", "module_mono_enabled=yes"])
subprocess.run(["scons", "target=template_release", "module_mono_enabled=yes"])


# Copy export template to target folder.
version_output = subprocess.run([GODOT_BIN, "--version"], capture_output=True, text=True).stdout
version = re.search(r'^(\d+\.\d+)\.dev.mono', version_output).group()
DEBUG_EXPORT_TEMPLATE_ORIGINAl_PATH = os.path.join(GODOT_OUTPUT_DIR, "godot.{0}.template_{1}.x86_64.mono.exe".format(PLATFORM,"debug"))
RELEASE_EXPORT_TEMPLATE_ORIGINAl_PATH = os.path.join(GODOT_OUTPUT_DIR, "godot.{0}.template_{1}.x86_64.mono.exe".format(PLATFORM,"release"))
TEMPLATE_PATH = os.path.expandvars(os.path.join("%USERPROFILE%", "AppData", "Roaming", "Godot", "export_templates", version))
DEBUG_EXPORT_TEMPLATE_TARGET_PATH = os.path.join(TEMPLATE_PATH, "{0}_{1}_x86_64.exe".format(PLATFORM,"debug"))
RELEASE_EXPORT_TEMPLATE_TARGET_PATH = os.path.join(TEMPLATE_PATH, "{0}_{1}_x86_64.exe".format(PLATFORM,"release"))
if not os.path.exists(TEMPLATE_PATH):
    os.makedirs(TEMPLATE_PATH)
    
shutil.copy(DEBUG_EXPORT_TEMPLATE_ORIGINAl_PATH, DEBUG_EXPORT_TEMPLATE_TARGET_PATH)
shutil.copy(RELEASE_EXPORT_TEMPLATE_ORIGINAl_PATH, RELEASE_EXPORT_TEMPLATE_TARGET_PATH)
