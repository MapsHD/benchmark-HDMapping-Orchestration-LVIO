# Helper image for Step 1 (common-bag preparation). No ROS: the bags are read and
# written with the pure-Python rosbags library.
#
# Built and used by prepare_data_step1.sh:
#   docker build -t lvio-prepare prepare_data_step1
#
# The scripts are mounted at run time (/app), so editing them needs no rebuild.
# Versions are pinned to the ones the converter was tested with.

FROM python:3.12-slim

RUN pip install --no-cache-dir \
        rosbags==0.11.9 \
        numpy==2.5.4 \
        opencv-python-headless==5.0.0.93 \
        PyYAML==6.0.2

WORKDIR /app
CMD ["python3", "/app/make_common_bag.py", "--help"]
