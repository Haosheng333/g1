"""orchestrator -- declarative sequencing for the grasp-and-deliver pipeline.

This package composes other modules' services and contains NO robotics of its
own: no IK, no TF maths, no path planning, no perception, no arm or hand
control. Each of those lives in its owning package and is reached only through
its published interface.

    routes          declarative route and step definitions (pure data)
    capabilities    what each step needs, and who owns it
    preflight       readiness evaluation over an injected probe
    nav_guard       the navigation saw_active terminal-state guard
    execution_lock  non-blocking single-route lock
    sequencer       step traversal over an injected backend (dry-run or real)
"""
