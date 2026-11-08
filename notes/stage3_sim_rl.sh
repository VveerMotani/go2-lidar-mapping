#!/usr/bin/env bash
# Stage 3: simulation + reinforcement learning stack
set -euo pipefail
WS="$HOME/unitree_ws"
mkdir -p "$WS/sim"

echo "### RL/sim virtualenv (isolated from ROS python)"
python3 -m venv --system-site-packages "$WS/venv-rl"
set +u; source "$WS/venv-rl/bin/activate"; set -u
pip install --upgrade pip setuptools wheel

echo "### PyTorch (CUDA 12.1, for the RTX 4070)"
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

echo "### MuJoCo + RL deps"
pip install mujoco mujoco-python-viewer glfw imageio scipy matplotlib \
            gymnasium tensorboard onnx onnxruntime pyyaml tqdm

echo "### unitree_mujoco"
cd "$WS/sim"
[ -d unitree_mujoco ] || git clone https://github.com/unitreerobotics/unitree_mujoco.git

echo "### unitree_rl_gym"
cd "$WS/sim"
[ -d unitree_rl_gym ] || git clone https://github.com/unitreerobotics/unitree_rl_gym.git

echo "### rsl_rl (PPO used by unitree_rl_gym)"
cd "$WS/sim"
[ -d rsl_rl ] || git clone https://github.com/leggedrobotics/rsl_rl.git -b v1.0.2
pip install -e "$WS/sim/rsl_rl" || true

deactivate || true
echo "### STAGE 3 COMPLETE"
