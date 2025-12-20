# Use Python 3.11 slim image as base
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        gcc \
        git \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first to leverage Docker cache
COPY backend/requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY backend/ ./

# Set default port (will be overridden by Railway)
ENV PORT=8000

# Expose port (using fixed value since EXPOSE doesn't support env vars in all contexts)
EXPOSE 8000

# Run the application with proper PORT handling
CMD ["sh", "-c", "uvicorn main:app --host 0.0.0.0 --port ${PORT}"]