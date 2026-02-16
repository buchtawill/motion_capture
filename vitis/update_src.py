#!/usr/bin/python3

import os
import shutil
import hashlib
import sys

# ==========================================
# CONFIGURATION
# ==========================================
APP_NAME = "zybo_ov9281_app"  # MUST match the app name in your build script/workspace

# ==========================================
# PATH SETUP
# ==========================================
script_dir = os.path.dirname(os.path.abspath(__file__))
# The "Golden" source of truth (your git repo)
golden_src_dir = os.path.join(script_dir, "src")
# The "Disposable" workspace where Vitis edits happen
workspace_src_dir = os.path.join(script_dir, "workspace", APP_NAME, "src")

def log(level, func_name, message):
    """Helper to enforce strict logging format."""
    print(f"{level} [update_src::{func_name}] {message}")

def calculate_md5(file_path):
    """Calculate MD5 hash of a file to check for content changes."""
    hash_md5 = hashlib.md5()
    try:
        with open(file_path, "rb") as f:
            for chunk in iter(lambda: f.read(4096), b""):
                hash_md5.update(chunk)
        return hash_md5.hexdigest()
    except FileNotFoundError:
        return None

def update_sources():
    func_name = "update_sources"
    
    log("INFO", func_name, "Starting synchronization: WORKSPACE -> GOLDEN SRC")
    log("INFO", func_name, f"Golden Source:    {golden_src_dir}")
    log("INFO", func_name, f"Workspace Source: {workspace_src_dir}")

    if not os.path.exists(workspace_src_dir):
        log("ERROR", func_name, f"Workspace directory not found at: {workspace_src_dir}")
        log("ERROR", func_name, "Have you built the Vitis project yet?")
        sys.exit(1)

    updated_count = 0
    skipped_count = 0
    error_count = 0
    
    # Walk through the GOLDEN directory.
    # We only care about files we are already tracking in the repo.
    for root, dirs, files in os.walk(golden_src_dir):
        for file in files:
            # 1. Get relative path (e.g., "ov5640/OV5640.cpp")
            #    This maps the golden structure to the workspace structure
            rel_path = os.path.relpath(os.path.join(root, file), golden_src_dir)
            
            # 2. Construct full paths
            golden_path = os.path.join(golden_src_dir, rel_path)
            workspace_path = os.path.join(workspace_src_dir, rel_path)

            # 3. Check if file exists in workspace
            if os.path.exists(workspace_path):
                # Calculate hashes to see if content actually changed
                golden_hash = calculate_md5(golden_path)
                workspace_hash = calculate_md5(workspace_path)

                if golden_hash != workspace_hash:
                    try:
                        shutil.copy2(workspace_path, golden_path)
                        log("INFO", func_name, f"Updated: {rel_path}")
                        updated_count += 1
                    except Exception as e:
                        log("ERROR", func_name, f"Failed to copy {rel_path}: {e}")
                        error_count += 1
                else:
                    # Content is identical
                    skipped_count += 1
            else:
                # This often happens for .gitignore, README, or helper scripts 
                # that you keep in src but Vitis doesn't use/copy.
                log("WARNING", func_name, f"File exists in Repo but NOT in Workspace: {rel_path}")

    log("INFO", func_name, "Sync Complete.")
    log("INFO", func_name, f"Files Updated:   {updated_count}")
    log("INFO", func_name, f"Files Unchanged: {skipped_count}")
    
    if error_count > 0:
        log("WARNING", func_name, f"Errors encountered: {error_count}")
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == "__main__":
    update_sources()