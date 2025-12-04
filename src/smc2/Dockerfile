FROM python:3.10-slim

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python libs
RUN pip install ur-rtde \
    pymodbus \
    numpy \
    scipy

WORKDIR /app

# Copy ALL project files
COPY . .

# Default command opens a shell so the user can choose what script to run
CMD ["/bin/bash"]
