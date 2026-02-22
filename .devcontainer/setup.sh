#!/bin/bash
set -e

# This script runs once after the devcontainer is created (postCreateCommand).
# It installs sources, generates assets, and builds the workspace.

echo "=== Post-create setup starting... ==="

# ── 1. Clone manipulator package if missing ───────────────────────────────────
if [ ! -d "src/open_manipulator_dynamixel" ]; then
    echo "[setup] Cloning open_manipulator_dynamixel..."
    git clone https://github.com/UruBots/open_manipulator_dynamixel.git src/open_manipulator_dynamixel
else
    echo "[setup] open_manipulator_dynamixel already present, skipping clone."
fi

# ── 2. Generate AprilTag PNG textures for Gazebo models ──────────────────────
# These are needed by limo_manipulation for simulation-based AprilTag detection.
TEXTURES_DIR="src/atwork_arena_description/materials/textures"
mkdir -p "$TEXTURES_DIR"

if [ ! -f "${TEXTURES_DIR}/apriltag_36h11_id0.png" ]; then
    echo "[setup] Generating AprilTag 36h11 PNG textures..."
    python3 << 'PYEOF'
import numpy as np
from PIL import Image
import os

TAG36H11_CODES = {
    0:  0x0000000d5d628584,
    1:  0x0000000d97f18b49,
    2:  0x0000000dd9952810,
    3:  0x00000010aba34933,
    4:  0x00000016d03750b5,
    5:  0x0000001cfd4e9c0c,
}

def generate_tag_image(code_value, tag_size_px=256, border=1):
    inner_cells = 6
    total_cells = inner_cells + 2 * border
    quiet_cells = 2
    full_cells  = total_cells + 2 * quiet_cells
    cell_px = tag_size_px // full_cells

    grid = np.zeros((full_cells, full_cells), dtype=np.uint8)
    b0, b1 = quiet_cells, quiet_cells + total_cells
    d0, d1 = b0 + border, b1 - border
    grid[b0:b1, b0:b1] = 1
    grid[d0:d1, d0:d1] = 0

    bits = [(code_value >> bit) & 1 for bit in range(35, -1, -1)]
    for r in range(inner_cells):
        for c in range(inner_cells):
            grid[d0 + r, d0 + c] = bits[r * inner_cells + c]

    pixels = ((1 - grid) * 255).astype(np.uint8)
    img_array = np.kron(pixels, np.ones((cell_px, cell_px), dtype=np.uint8))
    return Image.fromarray(img_array, mode='L').convert('RGB')

out_dir = 'src/atwork_arena_description/materials/textures'
for tag_id, code in TAG36H11_CODES.items():
    path = os.path.join(out_dir, f'apriltag_36h11_id{tag_id}.png')
    generate_tag_image(code).save(path)
    print(f'  Generated {path}')
PYEOF
    echo "[setup] AprilTag textures generated."
else
    echo "[setup] AprilTag textures already exist, skipping generation."
fi

# ── 3. rosdep update & install ───────────────────────────────────────────────
echo "[setup] Running rosdep update..."
rosdep update

echo "[setup] Installing rosdep dependencies..."
sudo rosdep install --from-paths src --ignore-src -y --rosdistro humble

# ── 4. Done ───────────────────────────────────────────────────────────────────
echo ""
echo "=== Setup complete! ==="
echo "Build the workspace with:"
echo "  cd \$WORKSPACE && colcon build --symlink-install"
echo "  source install/setup.bash"
