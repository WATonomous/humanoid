# Isaac Sim + IsaacLab on SixtyTwo AI — Setup Guide

A step-by-step guide to renting a cloud GPU on SixtyTwo AI, installing Isaac Sim and IsaacLab, viewing the GUI over VNC, and running demo/tutorial scripts.

---

## 1. Install the SixtyTwo CLI (on your local machine)

```bash
pip install --upgrade sixtytwo-cli
```

If the `sixtytwo` command isn't found after install (common on Windows), run it as a module instead:
```bash
python -m sixtytwo
```

Log in — this opens a browser auth flow and confirms with your account email.

## 2. Rent a GPU and connect

From the SixtyTwo dashboard, start a rental. You'll get an SSH command like:
```bash
ssh root@ssh5.vast.ai -p 38832
```

Connect using the SSH key SixtyTwo generated for you:
```bash
ssh -i ~/Downloads/sixtytwo_ed25519 root@ssh5.vast.ai -p 38832
```

> Your rental is already a Docker container on the provider's host — you're inside it as soon as you connect. No extra Docker steps needed.

## 3. Set up a Conda environment

```bash
conda create -n isaacsim python=3.11 -y
```

If you hit a `CondaToSNonInteractiveError`, accept the Terms of Service first:
```bash
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

Activate it:
```bash
conda activate isaacsim
```

## 4. Verify the GPU is visible

```bash
nvidia-smi
```

Confirm your GPU, VRAM, and driver/CUDA version show up correctly before continuing.

## 5. Install Isaac Sim

```bash
pip install isaacsim[all,extscache]==5.0.0 --extra-index-url https://pypi.nvidia.com
```

## 6. Use tmux to keep things running

SixtyTwo/vast.ai auto-attaches your SSH session to a tmux session. Use it instead of opening extra SSH connections (a second SSH session can conflict with or fail to reach the first).

- New window: `Ctrl+b` then `c`
- Next / previous window: `Ctrl+b` then `n` / `p`
- List windows: `Ctrl+b` then `w`
- Split into panes: `Ctrl+b` then `%` (vertical) or `"` (horizontal)
- Move between panes: `Ctrl+b` then an arrow key
- Detach without killing anything: `Ctrl+b` then `d`
- Reattach after reconnecting: `tmux attach -t <session-name>`
- Scroll back in the current pane: `Ctrl+b` then `[`, use arrow keys/mouse wheel, press `q` to exit

**Important:** `export DISPLAY=:1` (see Step 7) does not carry over between tmux windows automatically — export it fresh in every window that needs to launch a GUI app.

## 7. Set up a virtual display + VNC (to see the Isaac Sim GUI)

Install the required packages:
```bash
apt update
apt install -y x11vnc xvfb novnc websockify
```

Start the virtual display, VNC server, and web bridge (each `&` backgrounds the process):
```bash
Xvfb :1 -screen 0 1280x720x24 &
sleep 2
export DISPLAY=:1
x11vnc -display :1 -forever -nopw -listen localhost -xkb &
sleep 2
websockify -D --web=/usr/share/novnc/ 6080 localhost:5900
```

## 8. Tunnel the VNC port to your laptop

From a terminal **on your laptop** (not inside your SSH session to the cloud box):
```bash
ssh -i ~/Downloads/sixtytwo_ed25519 -L 6080:localhost:6080 root@ssh5.vast.ai -p 38832
```

Leave this terminal open. Then, in your browser:
```
http://localhost:6080/vnc.html
```
Click **Connect** (no password needed).

## 9. Launch Isaac Sim's GUI app

In a tmux window with `DISPLAY=:1` exported:
```bash
export DISPLAY=:1
export OMNI_KIT_ALLOW_ROOT=1
isaacsim isaacsim.exp.full
```

Wait for `Isaac Sim Full App is loaded.` in the log — this can take 1–3 minutes on the first run while extensions sync. Watch it appear in your VNC browser tab.

## 10. Speed up git for a slow/flaky connection

Cloud instance networking can be inconsistent for large `git clone` operations. Set these once, before installing IsaacLab:
```bash
git config --global http.postBuffer 524288000
git config --global http.version HTTP/1.1
```

If a clone times out anyway, retry with `--depth 1` (shallow clone — much less data to transfer):
```bash
git clone --depth 1 <repo-url>
```

## 11. Install IsaacLab

```bash
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
conda activate isaacsim
./isaaclab.sh --install
```

This installs IsaacLab's RL frameworks and extensions into your existing `isaacsim` conda environment. It can take 10–15 minutes.

## 12. Always use `--experience isaacsim.exp.full.kit` when running scripts

Without this flag, some IsaacLab scripts fail with:
```
ModuleNotFoundError: No module named 'omni.kit.usd'
```

Fix — always pass the flag explicitly:
```bash
./isaaclab.sh -p <script_path> --experience isaacsim.exp.full.kit
```

## 13. Fix the Nucleus/asset-root resolution error

Some scripts fail with:
```
FileNotFoundError: Unable to open the usd file at path: None/Isaac/Environments/Grid/default_environment.usd
```

This happens because IsaacLab's asset-root resolver does a folder existence check (`omni.client.stat`) against NVIDIA's asset bucket, which fails as a false negative on raw S3 URLs (even though the individual files are reachable). The fix is to hardcode the working URL directly in IsaacLab's source, bypassing that check:

```bash
grep -n "NUCLEUS_ASSET_ROOT_DIR = " ~/IsaacLab/source/isaaclab/isaaclab/utils/assets.py
```

Confirm it's on line 30, then replace it:
```bash
sed -i '30s|.*|NUCLEUS_ASSET_ROOT_DIR = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"|' ~/IsaacLab/source/isaaclab/isaaclab/utils/assets.py
```

Verify the edit:
```bash
sed -n '28,37p' ~/IsaacLab/source/isaaclab/isaaclab/utils/assets.py
```

## 14. Run tutorial and demo scripts

With `DISPLAY=:1` exported and the fixes above in place:
```bash
export DISPLAY=:1
cd ~/IsaacLab
./isaaclab.sh -p scripts/tutorials/01_assets/run_articulation.py --experience isaacsim.exp.full.kit
```

```bash
./isaaclab.sh -p scripts/demos/arms.py --experience isaacsim.exp.full.kit
```

Watch the results in your VNC browser tab (`http://localhost:6080/vnc.html`).

## 15. Run headless RL training (no GUI needed)

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Cartpole-v0 --headless
```

---

## Quick troubleshooting reference

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError: No module named 'omni.kit.usd'` | Add `--experience isaacsim.exp.full.kit` to the launch command |
| `Unable to open the usd file at path: None/...` | Patch `NUCLEUS_ASSET_ROOT_DIR` (Step 13) |
| `Cannot set window title without a default window` / no GUI shows | Xvfb isn't running — restart Steps 7 in order before launching Isaac Sim |
| `Address already in use` on port 6080 | A previous `websockify`/`x11vnc`/`Xvfb` is already running — check with `ps aux \| grep -E "websockify\|x11vnc\|Xvfb"` before starting new ones, or `pkill -9` the old ones first |
| VNC page stuck on "Connect" | Your SSH tunnel (Step 8) isn't open, or Xvfb/x11vnc/websockify died — check both sides |
| `git clone` times out / `Connection timed out` | Use `git config --global http.postBuffer 524288000` + `http.version HTTP/1.1`, and retry with `--depth 1` |
