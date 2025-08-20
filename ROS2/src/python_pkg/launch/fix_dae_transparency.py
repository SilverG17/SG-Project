import os

def fix_transparency_in_dae(folder_path):
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.dae'):
                file_path = os.path.join(root, file)
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Replace transparency value
                new_content = content.replace('<float>1.0</float>', '<float>0.0</float>')

                # Only overwrite if changes were made
                if new_content != content:
                    with open(file_path, 'w', encoding='utf-8') as f:
                        f.write(new_content)
                    print(f"✔ Fixed: {file_path}")
                else:
                    print(f"➖ No change needed: {file_path}")

# 🔧 Replace this with your actual mesh folder path
mesh_folder = 'AGV/src/python_pkg/urdf/meshes'
fix_transparency_in_dae(mesh_folder)
