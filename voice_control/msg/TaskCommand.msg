# /voice/command  --  one confirmed spoken task, for the orchestrator.
#
# Published by voice_control_node after the person confirmed the task (or the
# cancel window closed). The orchestrator subscribes, runs the named route
# under its execution lock and answers on /orchestrator/command_result with a
# TaskResult carrying the same `seq`. Mirrors g1_intellect, where main_process
# subscribes to g1_voice_intent's /voice/intent.
uint32 seq            # per-command id, echoed in the result
string command_id     # orchestrator route: fetch_deliver | grasp | deliver
string target         # pickup standing waypoint (fetch_deliver) or the route's target
string destination    # delivery standing waypoint (fetch_deliver), else ""
string label          # perception class (fetch_deliver), else ""
string summary        # what was understood, in words, for the log
string transcript     # the utterance as recognised
