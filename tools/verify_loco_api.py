#!/usr/bin/env python3
"""ON THE ROBOT PC, robot does NOT move: check that the installed unitree_sdk2py has the walking API that
navigation/scripts/locomotion_bridge.py calls.  Run it BEFORE ever starting the bridge with dry_run:=false.

    python3 tools/verify_loco_api.py [network_interface]      # e.g. enp2s0; without it only the import/signature check runs
"""
import inspect
import sys

ok_all = True


def check(name, ok, detail=""):
    global ok_all
    ok_all &= bool(ok)
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, ("  -- " + detail) if detail else ""))


def main():
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: F401
        check("import unitree_sdk2py.core.channel", True)
    except Exception as exc:  # noqa: BLE001
        check("import unitree_sdk2py.core.channel", False, "%s (pip install unitree_sdk2py in THIS python)" % exc)
        return 1
    try:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        check("import unitree_sdk2py.g1.loco.g1_loco_client.LocoClient", True)
    except Exception as exc:  # noqa: BLE001
        check("import ...g1_loco_client.LocoClient", False,
              "%s  -> find the real module with: python3 -c \"import unitree_sdk2py,pkgutil;"
              "print([m.name for m in pkgutil.walk_packages(unitree_sdk2py.__path__,'unitree_sdk2py.')])\" "
              "and fix DdsLocoClient in locomotion_bridge.py" % exc)
        return 1
    for meth, nargs in (("Init", 0), ("SetTimeout", 1), ("Move", 3), ("StopMove", 0)):
        fn = getattr(LocoClient, meth, None)
        check("LocoClient.%s exists" % meth, callable(fn))
        if callable(fn):
            params = [p for p in inspect.signature(fn).parameters.values() if p.name != "self"]
            required = [p for p in params if p.default is inspect.Parameter.empty]
            check("LocoClient.%s accepts %d positional args" % (meth, nargs),
                  len(params) >= nargs and len(required) <= nargs, str(inspect.signature(fn)))
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
        c = LocoClient()
        c.SetTimeout(5.0)
        c.Init()
        check("loco service reachable on %s (Init() returned)" % sys.argv[1], True,
              "nothing was commanded; the robot did not move")
    else:
        print("[skip] no network interface given: not contacting the robot")
    print("\nRESULT:", "ALL PASS" if ok_all else "FAILED")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
