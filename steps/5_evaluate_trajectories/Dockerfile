# Helper image lvio-eval for Step 5 (trajectory errors with evo) and Step 6 (coverage).
# No ROS: the inputs are the TUM files of Step 4, the ground-truth csv files and the run configs.
#
# Built and used by evaluate_trajectories.sh and steps/6_overlap/overlap.sh:
#   docker build -t lvio-eval steps/5_evaluate_trajectories
#
# The scripts are mounted at run time (/app, /lvio_py), so editing them needs no rebuild.
# evo is pinned; its dependencies (numpy, scipy, matplotlib, pandas, ...) are resolved by pip,
# with numpy and PyYAML pinned to the versions of the lvio-prepare image.

FROM python:3.12-slim

RUN pip install --no-cache-dir \
        evo==1.38.1 \
        numpy==2.5.4 \
        PyYAML==6.0.2 && \
    pip freeze > /pip-freeze.txt

ENV MPLBACKEND=Agg \
    MPLCONFIGDIR=/tmp/matplotlib \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app
CMD ["python3", "/app/evaluate_trajectories.py", "--help"]
