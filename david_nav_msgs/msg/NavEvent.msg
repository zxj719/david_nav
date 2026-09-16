# Observation only: never a task, cancellation or movement command.
# Producers assign a unique ID once and preserve it when forwarding/replaying.
string event_id
# Finite Unix wall-clock seconds, matching events.jsonl; not ROS/simulation time.
float64 stamp
# Producing component. Ingress source/request IDs belong in data_json.
string source
# Maps to the events.jsonl "type" key.
string event_type
# Application-validated: info | warn | error.
string severity
# A JSON object, never NaN/Infinity; SI units (metres, radians, seconds).
string data_json
