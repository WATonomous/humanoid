# Quest Bimanual Teleop (CloudXR + Pink IK)

Hand-tracking teleop of the bimanual arm on a Quest, streamed via CloudXR.

## 1. Start the USB tunnel (gnirehtet)

The Quest reaches the PC through a USB reverse-tether. Plug the Quest in over USB,
then:

```bash
cd ~/gnirehtet/gnirehtet-rust-linux64 && ./gnirehtet run
```

Approve the **VPN prompt on the headset** (a key icon appears). Leave this
running.

> If you restart `adb` or replug the Quest, the reverse mapping gnirehtet needs
> gets wiped. Re-add it (then relaunch the client / re-run `./gnirehtet run`):
>
> ```bash
> adb reverse tcp:31416 tcp:31416
> ```

## 2. Run the teleop

```bash
cd ~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop && ./run_bimanual_teleop.sh
```

Wait for `IsaacTeleop teleoperation started`. It runs headless — the view only
appears in the headset.

Add `--gui` for a local desktop window (no headset/CloudXR).

## 3. Connect the headset

In the Quest browser open `https://<PC_IP>:48322/`, accept the self-signed cert,
then in the CloudXR client set Server IP `<PC_IP>`, Port `48322`, **VR Immersive**,
and **Connect**.

Move your hands to drive the arms; pinch thumb + index to close each gripper.
