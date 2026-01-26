# Pause/unpause command for behavior tree execution
# When paused=true, execution should pause
# When paused=false, execution should resume

bool paused
string source      # Source of the pause command (e.g., "operator", "safety_system", "low_battery")
string reason      # Human-readable reason for the pause
