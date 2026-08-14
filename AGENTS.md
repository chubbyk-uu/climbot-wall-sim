# Climbot Sim Agent Instructions

These instructions apply to all work in this repository.

## Long-running test output

- Run long Gazebo, ROS 2 launch, Action, and `colcon test` jobs in the background or
  in a persistent execution session so the agent can wait for completion without
  repeatedly streaming output.
- Redirect complete stdout and stderr to a uniquely named log under `/tmp`, such as
  `/tmp/climbot_<case>_<timestamp>.log`. Temporary test logs must not be committed.
- On success, read and report only the process exit code, the concise test/result
  summary, key acceptance metrics, and the log path. Do not load the complete log
  into the model context.
- On failure, search the saved log first with `rg` for relevant markers such as
  `FAILED`, `ERROR`, `FATAL`, exceptions, non-zero result codes, and timeout or
  abort messages. Read only the small surrounding sections needed to diagnose the
  problem; do not dump the entire log unless explicitly requested.
- For long Gazebo integration tests, keep simulator, controller, evaluator, and
  Action-client logs separate when practical. Extract only the final Action result,
  safety-stop state, trajectory metrics, and process cleanup status.
- Save artifacts under `results/` only when they are formal, reproducible project
  acceptance evidence. Routine diagnostic logs remain under `/tmp`.
- Do not claim a background test has started until its process or persistent
  execution session has actually been created.

