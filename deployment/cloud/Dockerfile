# UNVERIFIED TEMPLATE: not built, not GPU-tested. Supply an independently pinned
# Python 3.12/CUDA image digest from the resolved deployment manifest.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
ARG BASE_IMAGE
RUN case "$BASE_IMAGE" in *@sha256:*) ;; *) echo "Pinned image digest required" >&2; exit 2;; esac
WORKDIR /workspace/Cable_robot
COPY vla_protocol ./vla_protocol
COPY vla_server ./vla_server
COPY deployment/cloud ./deployment/cloud
RUN python3 -m venv /opt/vla \
 && /opt/vla/bin/python -m pip install --no-deps -r deployment/cloud/requirements-server.lock \
 && /opt/vla/bin/python -m pip install --no-build-isolation --no-deps ./vla_protocol ./vla_server
# LeRobot/checkpoints/verified GPU lock are separate mounted artifacts, never COPY weights.
ENV HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
ENV VLA_PYTHON=/opt/vla/bin/python
# GPU bootstrap additionally requires a read-only mounted, clean Git checkout
# at this WORKDIR so the source revision can be verified independently.
ENTRYPOINT ["bash", "deployment/cloud/scripts/start_server.sh"]
CMD ["--config", "deployment/cloud/config/mock.yaml"]
