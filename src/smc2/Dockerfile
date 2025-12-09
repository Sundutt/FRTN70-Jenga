FROM python:3.10-slim

# Install OS dependencies required for OpenCV + ArUco + Video + UI + Mediapipe
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libgl1 \
    libglib2.0-0 \
    libopencv-dev \
    libsm6 \
    libxext6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries
RUN pip install \
    opencv-contrib-python \
    numpy \
    scipy \
    ur-rtde \
    pymodbus \
    mediapipe

WORKDIR /app

# Copy project to container
COPY . .

# Default: enter interactive shell
CMD ["/bin/bash"]
