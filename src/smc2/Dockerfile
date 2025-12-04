FROM python:3.10-slim

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python libs
RUN pip install ur-rtde 
RUN pip install pymodbus numpy scipy

WORKDIR /app

# Copy ALL Python files
COPY . .

CMD ["python3", "control_test.py"]
