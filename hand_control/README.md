# hand_control (Module 3.2)

BrainCo Revo2 hand as three `std_srvs/Trigger` services per side:

| Service | Action |
|---|---|
| `/hand_{left,right}/pre_grasp` | open / pre-shape the fingers |
| `/hand_{left,right}/grasp` | close the fingers (caller retries 3 x 0.4 s) |
| `/hand_{left,right}/release` | open to release |

All finger numbers live in `config/finger_presets.yaml`
(per-mille, **0 = open, 1000 = closed**, order `[thumb, thumb_aux, index, middle, ring, pinky]`).

## Install
    pip3 install -r requirements.txt            # on Noetic (Python 3.8): pip3 install "bc-stark-sdk<2"
    sudo usermod -aG dialout $USER              # then log out / in
    cd ~/catkin_ws/src && cp -r <this folder> hand_control && cd .. && catkin_make && source devel/setup.bash

Check the SDK has the functions this package uses (prints [] if all present):
    python3 -c "from bc_stark_sdk import main_mod as m; print([n for n in ('auto_detect','init_from_detected','close_device_handler','HandType') if not hasattr(m,n)])"

## Run
    roslaunch hand_control hand.launch side:=left                          # real hand, auto-detect port
    roslaunch hand_control hand.launch side:=left com_port:=/dev/ttyUSB0   # force a port
    roslaunch hand_control hand.launch side:=left sim:=true with_bottle:=true   # MuJoCo (pip3 install mujoco)

## Try it (hand empty first, hands clear of the fingers)
    rosservice call /hand_left/pre_grasp
    rosservice call /hand_left/grasp
    rosservice call /hand_left/release

## Behaviour worth knowing
* The node never moves the hand at startup or shutdown (restarting must not drop a held bottle).
  Call `release` yourself first if you are about to stop it while holding something.
* `grasp` closes in 5 steps (`ramp_steps` / `ramp_dt`), not in one jump.
* `grasp` returns success even though fingers stop short (the bottle blocks them). To make it FAIL when
  nothing is in the hand, set `expect_contact: true` for `grasp` once you've seen real feedback with a bottle.
* A failed serial command triggers one automatic reconnect + one retry; after that the service returns
  `success=False` with the reason (the orchestrator's retries take over).
* The YAML is validated at startup (6 values, 0..1000, required presets, unknown keys rejected).
* The old `force_limit` / `speed` keys were removed: they were never sent to the hand.
