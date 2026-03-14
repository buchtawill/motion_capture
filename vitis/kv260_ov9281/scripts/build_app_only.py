###############################################################################
# For help building vitis scripts, manually create a project in vitis IDE and
# view the build.py under logs
###############################################################################

import vitis
import os
import sys
import shutil
import json
import signal

def die_with_error(message):
    print(f"ERROR [build_app_only.py] {message}")
    try:
        vitis.dispose() # type: ignore
    except Exception as e:
        print(f"Error disposing vitis: {e}")
    sys.exit(1)

def signal_handler(sig, frame):
    print("\nINFO [build_app_only.py] Caught Ctrl+C, disposing vitis...")
    try:
        vitis.dispose() # type: ignore
    except Exception as e:
        print(f"Error disposing vitis: {e}")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Create platform from XSA
# Create app from platform
# Build platform
# Import sources
# Build app

PROJECT_NAME = 'kv260_ov9281'
PLATFORM_NAME = f'{PROJECT_NAME}_platform'
APP_NAME = f'{PROJECT_NAME}_app'

# -----------------------------------------------------------------------------------
# 1. Setup Paths
# -----------------------------------------------------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(script_dir, "../src")
xsa_path = os.path.abspath(os.path.join(script_dir, f"../../../vivado/{PROJECT_NAME}/{PROJECT_NAME}_proj/{PROJECT_NAME}_proj.xsa"))
workspace_path = os.path.join(script_dir, "../workspace")

# -----------------------------------------------------------------------------------
# Clean existing application
# -----------------------------------------------------------------------------------
app_path = f"{workspace_path}/{APP_NAME}"
if os.path.exists(app_path):
    print(f"INFO [build_app_only.py] Cleaning old application at {app_path}...")
    shutil.rmtree(app_path)

# -----------------------------------------------------------------------------------
# 2. Initialize Vitis Client
# -----------------------------------------------------------------------------------
print("INFO [build_app_only.py] Creating vitis client")
client = vitis.create_client() # type: ignore
client.set_workspace(path=workspace_path)


# -----------------------------------------------------------------------------------
# 4. Create Application Component
# -----------------------------------------------------------------------------------
# We need the absolute path to the generated .xpfm file
platform_xpfm = os.path.join(workspace_path, f"{PLATFORM_NAME}/export/{PLATFORM_NAME}/{PLATFORM_NAME}.xpfm")
if not os.path.exists(platform_xpfm):
    print(f"ERROR: Platform XPFM file was not generated at: {platform_xpfm}")
    die_with_error("Platform XPFM file not generated")
    
print("INFO [build_app_only.py] Creating Application")
app_comp = client.create_app_component(
    name=APP_NAME,
    platform=platform_xpfm,
    domain='standalone_psu_cortexa53_0',
    template='empty_application'
)

# -----------------------------------------------------------------------------------
# 5. Import Sources
# -----------------------------------------------------------------------------------
print(f"INFO [build_app_only.py] Importing sources from {src_dir} to vitis project")
vitis_src_dir = os.path.join(workspace_path, APP_NAME, "src")

# Clean up specific template files you want to replace/remove
files_to_cleanup = ['Empty_applicationExample.cmake', 'CMakeLists.txt', 'UserConfig.cmake']

for filename in files_to_cleanup:
    file_path = os.path.join(vitis_src_dir, filename)
    if os.path.exists(file_path):
        os.remove(file_path)
        print(f"   - Removed template file: {filename}")

# Copy and merge sources
if os.path.exists(src_dir):
    shutil.copytree(src_dir, vitis_src_dir, dirs_exist_ok=True)
    print(f"   - Successfully merged sources from {src_dir}")
else:
    print(f"ERROR [build_app_only.py] Source directory {src_dir} not found!")
    die_with_error("Source directory not found")

# -----------------------------------------------------------------------------------
# 6. Build
# -----------------------------------------------------------------------------------
print(f"INFO [build_app_only.py] Building application")
result = app_comp.build()

if(result != 0):
    print("ERROR [build_app_only.py] Application build failed")
    die_with_error("Application build failed")
print(f"INFO [build_app_only.py] Application build successful")


vitis.dispose() # type: ignore
sys.exit(0)