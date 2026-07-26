# The follower + its watchdog and health watcher. The SDK server (Chrome + Agora)
# runs in its own container from the frodobots-org/earth-rovers-sdk repo — see
# DEPLOYMENT.md; do not vendor it here, so `git pull` there is all an update takes.
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY *.py ./
COPY vision/ ./vision/

ENV SDK_BASE_URL=http://sdk:8000 \
    HEARTBEAT_PATH=/tmp/erc_follower.hb \
    PYTHONUNBUFFERED=1

CMD ["python", "waypoint_follower.py", "--watchdog", "--log", "/logs/run.csv"]
