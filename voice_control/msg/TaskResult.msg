# /orchestrator/command_result  --  the orchestrator's answer to one TaskCommand.
uint32   seq            # the TaskCommand.seq this answers
string   command_id
bool     accepted       # false = refused before running (busy, unknown route, bad request)
bool     success
string   message
string[] blockers
bool     dry_run
bool     motion_issued
string   final_step
