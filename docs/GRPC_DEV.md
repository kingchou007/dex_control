# gRPC Development Guide

This project uses [gRPC](https://grpc.io/) + [Protocol Buffers](https://protobuf.dev/) for communication between the robot server (NUC) and client (laptop). This document explains how to modify, extend, and regenerate the proto definitions.

## Architecture

```
Laptop (Client)                          NUC (Server)
┌──────────────────┐     gRPC/HTTP2     ┌──────────────────┐
│ FrankaRobotClient│ ◄────────────────► │ FrankaServicer   │
│  (robot_client.py│                    │  (franka_wrapper │
│   uses stub)     │                    │   .py)           │
└──────────────────┘                    └──────────────────┘
        │                                        │
        ▼                                        ▼
  franka_pb2_grpc.py                      franka_pb2_grpc.py
  franka_pb2.py                           franka_pb2.py
        │                                        │
        └──────── generated from ────────────────┘
                         │
                   franka.proto
```

## File Layout

```
dex_control/proto/
├── franka.proto          # Source of truth — edit this
├── franka_pb2.py         # Generated — message classes (DO NOT EDIT)
├── franka_pb2_grpc.py    # Generated — stub & servicer (DO NOT EDIT)
└── __init__.py
```

## Prerequisites

```bash
pip install grpcio grpcio-tools
```

## How to Add a New RPC Method

### Step 1: Edit `franka.proto`

Add the new RPC to the `FrankaRobot` service and define any new messages.

**Example:** Adding a `GetCartesianVelocity` method:

```protobuf
service FrankaRobot {
  // ... existing RPCs ...

  // New: query cartesian velocity
  rpc GetCartesianVelocity (Empty) returns (CartesianVelocity);
}

// New message
message CartesianVelocity {
  repeated double linear = 1;   // [vx, vy, vz] in m/s
  repeated double angular = 2;  // [wx, wy, wz] in rad/s
}
```

### Step 2: Regenerate Python code

From the **project root**:

```bash
python -m grpc_tools.protoc \
  -I dex_control/proto \
  --python_out=dex_control/proto \
  --grpc_python_out=dex_control/proto \
  dex_control/proto/franka.proto
```

**Important:** After regeneration, fix the import in `franka_pb2_grpc.py`:

```python
# Change this (generated default):
import franka_pb2 as franka__pb2

# To this (package-relative):
from dex_control.proto import franka_pb2 as franka__pb2
```

This is needed because we import from `dex_control.proto`, not from the proto directory directly.

### Step 3: Implement on the server

In `dex_control/robot/franka_wrapper.py`:

1. Add the method to `FrankaWrapper` (the actual robot logic):

```python
def get_cartesian_velocity(self) -> Dict[str, List[float]]:
    state = self.robot.state
    return {
        "linear": list(state.O_dP_EE_c[:3]),
        "angular": list(state.O_dP_EE_c[3:]),
    }
```

2. Add the handler to `FrankaServicer` (the gRPC bridge):

```python
def GetCartesianVelocity(self, request, context):
    vel = self.w.get_cartesian_velocity()
    return franka_pb2.CartesianVelocity(
        linear=vel["linear"], angular=vel["angular"],
    )
```

### Step 4: Add to the client

In `dex_control/robot/robot_client.py`:

```python
def get_cartesian_velocity(self) -> Dict[str, np.ndarray]:
    """Get current end-effector velocity."""
    resp = self._call("GetCartesianVelocity")
    return {
        "linear": np.array(resp.linear),
        "angular": np.array(resp.angular),
    }
```

The `_call` method handles retry and reconnection automatically.

### Step 5: Add a test

In `tests/test_grpc.py`:

1. Add the method to `MockServicer`:

```python
def GetCartesianVelocity(self, request, context):
    return franka_pb2.CartesianVelocity(
        linear=[0.01, 0.0, 0.0], angular=[0.0, 0.0, 0.0],
    )
```

2. Add the test:

```python
def test_get_cartesian_velocity(self, server_and_client):
    client, _ = server_and_client
    vel = client.get_cartesian_velocity()
    assert "linear" in vel and "angular" in vel
    assert len(vel["linear"]) == 3
```

3. Run tests:

```bash
pytest tests/test_grpc.py -v
```

## Proto Style Guide

- **Service methods**: `PascalCase` (e.g., `GetJointPositions`, `MoveEePose`)
- **Messages**: `PascalCase` (e.g., `MoveEePoseRequest`, `BoolResponse`)
- **Fields**: `snake_case` (e.g., `target_pose`, `epsilon_inner`)
- **Use `repeated double`** for variable-length numeric arrays (joints, poses)
- **Reuse common messages** like `Empty`, `BoolResponse`, `FloatResponse`, `JointValues` when possible
- **Group related RPCs** with comments in the service definition

## Common Patterns

### Request with no input → use `Empty`

```protobuf
rpc GetJointPositions (Empty) returns (JointValues);
```

### Command that returns success/failure → use `BoolResponse`

```protobuf
rpc StopMotion (Empty) returns (BoolResponse);
```

### Query that returns a single number → use `FloatResponse`

```protobuf
rpc GetGripperWidth (Empty) returns (FloatResponse);
```

### Complex request → define a dedicated message

```protobuf
message GraspRequest {
  double width = 1;
  double speed = 2;
  double force = 3;
  // ...
}
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ModuleNotFoundError: franka_pb2` | Fix the import in `franka_pb2_grpc.py` (see Step 2) |
| `grpcio version mismatch` | `pip install --upgrade grpcio grpcio-tools` |
| Pylance shows `franka_pb2` errors | Normal — protobuf generated code lacks `.pyi` stubs. Runtime works fine. Optionally install `mypy-protobuf` to generate type stubs |
| Client can't connect | Check address format is `ip:port` (not `tcp://ip:port`) |
| Changes not reflected | Did you regenerate? Did you fix the import? Restart the server |
