import os
import re

# Get the absolute path to the SDF file
workspace_root = os.path.dirname(os.path.abspath(__file__))  # location of the script
sdf_path = os.path.join(workspace_root, 'urdf', 'test.sdf')

# Confirm the file exists
if not os.path.exists(sdf_path):
    raise FileNotFoundError(f"❌ SDF file not found at: {sdf_path}")

# Read and patch the file
with open(sdf_path, 'r', encoding='utf-8') as f:
    sdf = f.read()

sdf = re.sub(r'model://python_pkg/urdf/meshes/', r'file://$(find python_pkg)/urdf/meshes/', sdf)

with open(sdf_path, 'w', encoding='utf-8') as f:
    f.write(sdf)

print(f"✅ SDF mesh paths patched in: {sdf_path}")
