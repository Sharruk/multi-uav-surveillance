FROM python:3.10-slim

# Install system dependencies needed for PyBullet and other simulation libraries
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install dependencies first (to leverage Docker cache)
COPY requirements.txt .
# Install CPU-only torch to save 3GB of CUDA libraries
RUN pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip install --no-cache-dir -r requirements.txt
RUN pip install --no-cache-dir ray[rllib]

# Copy the rest of the application
COPY . .

# Set default entrypoint to training (can be overridden to run drone_env.py headless)
CMD ["python", "train.py"]
