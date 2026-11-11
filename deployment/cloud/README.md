# Stage 2 boundary (not implemented)

The future GPU service will install Cable_robot's `vla_protocol` and `vla_server`
in a separate environment and implement `vla_server.policy_base.Policy`.
Keep a pinned LeRobot source checkout and checkpoint directory outside Cable_robot.
Its Python/CUDA/Torch versions need not match ROS Humble's Python 3.10.

Use official preprocessing/postprocessing. Return native channels, units,
normalization and processor identities. The Stage 1 synthetic mapping intentionally
rejects real-model outputs until an explicit compatible mapping is implemented.
No weights, GPU adapter or training code is included in Stage 1.
