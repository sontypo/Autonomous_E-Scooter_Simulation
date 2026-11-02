#!/usr/bin/env python3
# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

"""Check that an exported policy stays inside its action range.

The action scales in EScooterActionsCfg assume the policy outputs land in
[-1, 1]: 0.7 rad of steering, 60 N.m of plate torque, 20 rad/s of wheel speed.
Nothing in training enforced that until `action_saturation_penalty` was added,
and the 2026-10-30 checkpoint does not respect it - its steering head reaches
a raw action of +7.3 once the reaction plate passes -150 rad/s, which pins the
steering at its mechanical limit and loses the scooter.

Run this before putting a policy anywhere near Gazebo. It needs only
onnxruntime on the CPU: no Isaac Sim, no GPU.

    python3 src/escooter_rl/scripts/check_policy_range.py
    python3 src/escooter_rl/scripts/check_policy_range.py --policy <file.onnx>

Exit status is 0 when every probe stays in range, 1 otherwise, so it works as
a gate in a script.
"""

import argparse
import json
import os
import sys

import numpy as np

ACTION_NAMES = ("steering", "reaction_plate", "drive_wheel")
WHEEL_RADIUS = 0.135  # m, from escooter_dynamics/params/escooter.yaml


def find_policy(start: str) -> str:
    """Newest .onnx under an exported/ directory in the repo's logs."""
    # <repo>/src/escooter_rl/scripts/this.py -> <repo>/src/escooter_rl
    pkg = os.path.dirname(os.path.dirname(os.path.realpath(start)))
    repo = os.path.dirname(os.path.dirname(pkg))
    best = None
    for base in (os.path.join(pkg, "logs", "rsl_rl"),
                 os.path.join(repo, "logs", "rsl_rl")):
        if not os.path.isdir(base):
            continue
        for dirpath, _dirs, files in os.walk(base):
            if os.path.basename(dirpath) != "exported":
                continue
            for name in files:
                if not name.endswith(".onnx"):
                    continue
                path = os.path.join(dirpath, name)
                if best is None or os.path.getmtime(path) > os.path.getmtime(best):
                    best = path
    return best


def load_spec(policy_path: str) -> dict:
    spec_path = os.path.join(os.path.dirname(policy_path), "obs_spec.json")
    if os.path.exists(spec_path):
        with open(spec_path) as handle:
            return json.load(handle)
    print("[WARN] no obs_spec.json beside the policy; assuming unscaled "
          "observations, which is how pre-spec policies were trained.")
    return {"task": None, "terms": []}


class ObsBuilder:
    """Builds observation vectors from PHYSICAL quantities.

    Applies the spec's clip and scale exactly as ObservationTermCfg does
    (noise -> clip -> scale), so a sweep is expressed in rad/s and m/s rather
    than in whatever units the network happens to see.
    """

    def __init__(self, spec: dict, obs_dim: int):
        terms = {t["name"]: t for t in spec.get("terms", [])}
        self.plate = terms.get("reaction_vel", {})
        self.wheels = terms.get("joint_vel", {})
        self.cmd_dim = obs_dim - 16

    @staticmethod
    def _prep(value, term):
        clip = term.get("clip")
        if clip is not None:
            value = float(np.clip(value, clip[0], clip[1]))
        return value * float(term.get("scale", 1.0))

    def __call__(self, v=(0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0), g=(0.0, 0.0, -1.0),
                 steer=0.0, plate=0.0, wheels=None, cmd=None, last=(0.0, 0.0, 0.0)):
        if cmd is None:
            cmd = (0.0,) * self.cmd_dim
        if wheels is None:
            # Wheel speed has to agree with body speed, or the probe describes
            # a locked-wheel skid that the robot cannot be in and the policy
            # was never shown. Leaving these at zero while setting v_x = 1.5
            # was what made an earlier version of this script report failures
            # the policy is not responsible for. Positive joint rotation drives
            # the scooter backwards, hence the sign.
            spin = -v[0] / WHEEL_RADIUS
            wheels = (spin, spin)
        return np.array([[
            *v, *w, *g, steer,
            self._prep(plate, self.plate),
            *[self._prep(x, self.wheels) for x in wheels],
            *cmd, *last,
        ]], dtype=np.float32)


def sweeps(build: ObsBuilder, cmd_dim: int):
    """(label, [observation, ...]) over the physically reachable envelope."""
    drive = (1.5,) + (0.0,) * (cmd_dim - 1)

    def rolled(deg):
        r = np.radians(deg)
        return (0.0, -np.sin(r), -np.cos(r))

    def pitched(deg):
        p = np.radians(deg)
        return (np.sin(p), 0.0, -np.cos(p))

    yield ("roll -35..35 deg",
           [build(g=rolled(d), v=(1.5, 0, 0), cmd=drive) for d in np.linspace(-35, 35, 71)])
    yield ("pitch -20..20 deg",
           [build(g=pitched(d), v=(1.5, 0, 0), cmd=drive) for d in np.linspace(-20, 20, 41)])
    yield ("plate -300..300 rad/s",
           [build(plate=p, v=(1.5, 0, 0), cmd=drive) for p in np.linspace(-300, 300, 121)])
    yield ("wheels -30..30 rad/s",
           [build(wheels=(x, x), v=(1.5, 0, 0), cmd=drive) for x in np.linspace(-30, 30, 61)])
    yield ("v_x -1..3 m/s",
           [build(v=(x, 0, 0), cmd=drive) for x in np.linspace(-1, 3, 41)])
    yield ("v_y -1..1 m/s",
           [build(v=(1.5, y, 0), cmd=drive) for y in np.linspace(-1, 1, 41)])
    yield ("roll rate -5..5 rad/s",
           [build(w=(x, 0, 0), v=(1.5, 0, 0), cmd=drive) for x in np.linspace(-5, 5, 41)])
    yield ("yaw rate -2..2 rad/s",
           [build(w=(0, 0, x), v=(1.5, 0, 0), cmd=drive) for x in np.linspace(-2, 2, 41)])
    yield ("steering -60..60 deg",
           [build(steer=np.radians(d), v=(1.5, 0, 0), cmd=drive)
            for d in np.linspace(-60, 60, 61)])
    # Command sweep with the body actually at the commanded speed: a standing
    # start is covered separately, and asking for 2.5 while rolling at 1.5 is a
    # transient the tracking sweep already covers.
    yield ("command v_x 0..2.5 m/s, tracked",
           [build(v=(x, 0, 0), cmd=(x,) + (0.0,) * (cmd_dim - 1))
            for x in np.linspace(0, 2.5, 26)])
    yield ("standing start, cmd 0..2.5",
           [build(v=(0, 0, 0), cmd=(x,) + (0.0,) * (cmd_dim - 1))
            for x in np.linspace(0, 2.5, 26)])
    yield ("last_action -1..1",
           [build(v=(1.5, 0, 0), cmd=drive, last=(a, a, a)) for a in np.linspace(-1, 1, 41)])


def random_box(build: ObsBuilder, cmd_dim: int, count: int, seed: int):
    """Everything varying at once. Axis sweeps miss interactions."""
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(count):
        roll, pitch = rng.uniform(-0.61, 0.61), rng.uniform(-0.35, 0.35)
        g = (np.sin(pitch),
             -np.sin(roll) * np.cos(pitch),
             -np.cos(roll) * np.cos(pitch))
        out.append(build(
            v=(rng.uniform(-0.5, 3.0), rng.uniform(-1, 1), 0.0),
            w=(rng.uniform(-5, 5), rng.uniform(-2, 2), rng.uniform(-2, 2)),
            g=g,
            steer=rng.uniform(-1.047, 1.047),
            plate=rng.uniform(-300, 300),
            # consistent with v_x, plus bounded slip rather than a free draw
            wheels=None,
            cmd=tuple(rng.uniform(-1, 2.5, cmd_dim)),
            last=tuple(rng.uniform(-1.5, 1.5, 3)),
        ))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", default=None,
                        help="Exported .onnx. Defaults to the newest under logs/rsl_rl.")
    parser.add_argument("--limit", type=float, default=1.0,
                        help="Largest acceptable |raw action|. Default 1.0.")
    parser.add_argument("--samples", type=int, default=20000,
                        help="Random observations to draw for the combined probe.")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    policy = args.policy or find_policy(__file__)
    if not policy or not os.path.exists(policy):
        print("[FAIL] no exported .onnx found; run scripts/export_onnx.py first.")
        return 1

    import onnxruntime as ort
    session = ort.InferenceSession(policy, providers=["CPUExecutionProvider"])
    inp = session.get_inputs()[0]
    obs_dim = inp.shape[1]

    spec = load_spec(policy)
    if spec.get("obs_dim") not in (None, obs_dim):
        print("[FAIL] obs_spec.json describes %d observations, the policy takes %d."
              % (spec["obs_dim"], obs_dim))
        return 1

    build = ObsBuilder(spec, obs_dim)
    print("policy   : %s" % policy)
    print("task     : %s   obs_dim: %s   limit: %.2f"
          % (spec.get("task"), obs_dim, args.limit))
    print()
    print("%-26s %8s %8s %8s   %s" % ("probe", *ACTION_NAMES, ""))

    worst = {name: 0.0 for name in ACTION_NAMES}
    failures = []

    def run(label, batch):
        out = session.run(None, {inp.name: np.concatenate(batch, axis=0)})[0]
        peak = np.max(np.abs(out), axis=0)
        for i, name in enumerate(ACTION_NAMES):
            worst[name] = max(worst[name], float(peak[i]))
        bad = peak > args.limit
        if bad.any():
            failures.append(label)
        print("%-26s %8.2f %8.2f %8.2f   %s"
              % (label, peak[0], peak[1], peak[2], "OUT OF RANGE" if bad.any() else "ok"))

    for label, batch in sweeps(build, build.cmd_dim):
        run(label, batch)
    run("random, %d samples" % args.samples,
        random_box(build, build.cmd_dim, args.samples, args.seed))

    print()
    print("worst |raw action|: " + "  ".join(
        "%s %.2f" % (name, worst[name]) for name in ACTION_NAMES))
    if failures:
        print()
        print("[FAIL] %d probe(s) left [-%.2f, %.2f]: %s"
              % (len(failures), args.limit, args.limit, ", ".join(failures)))
        print("       The action scales assume raw actions in that range. Raise")
        print("       action_saturation's weight in escooter_env_cfg.py and")
        print("       retrain; do not deploy this policy.")
        return 1
    print()
    print("[PASS] every probe stayed inside [-%.2f, %.2f]." % (args.limit, args.limit))
    return 0


if __name__ == "__main__":
    sys.exit(main())
