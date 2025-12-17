---
title: Deployment Guide
---

## Overview
This guide describes how to deploy the Physical AI Book RAG Chatbot to production.

## Prerequisites
- Python 3.9+
- Access to OpenRouter API
- Qdrant Cloud account (or self-hosted Qdrant instance)
- Neon Postgres account (or PostgreSQL database)
- Web server to serve frontend files (Nginx, Apache, etc.)

## Environment Configuration

Create a `.env` file with the following variables:

```bash
# OpenRouter API Configuration
OPENROUTER_API_KEY=your_openrouter_api_key_here
OPENROUTER_MODEL_NAME=openai/gpt-4-mini

# Qdrant Vector Database Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_content

# Neon Postgres Database Configuration
DATABASE_URL=your_neon_postgres_connection_string_here

# Application Configuration
MAX_CONTEXT_TOKENS=2000
SESSION_RETENTION_DAYS=30
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=3600
```

## Backend Deployment

### Option 1: Using Docker
1. Create a `Dockerfile`:
```Dockerfile
FROM python:3.9-slim

WORKDIR /app

COPY backend/requirements.txt .
RUN pip install -r requirements.txt

COPY backend/ .

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

2. Build and run the container:
```bash
docker build -t rag-chatbot-backend .
docker run -d -p 8000:8000 --env-file .env rag-chatbot-backend
```

### Option 2: Direct Installation
1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Initialize the database:
```bash
python initialize_db.py
```

3. Start the application:
```bash
uvicorn main:app --host 0.0.0.0 --port 8000
```

## Frontend Deployment

The frontend is designed to work with static site hosting:

1. Upload all files in the `frontend/` directory to your static file hosting service
2. Update the API base URL in `chat-interface.js` to point to your deployed backend
3. Ensure your hosting service serves `index.html` for the main page

## Database Setup

### Initialization
The application will automatically create database tables on startup. For initial content ingestion:

```bash
python initialize_db.py
```

### Migrations
The application uses Alembic for database migrations:
```bash
# Create a new migration
alembic revision --autogenerate -m "description of changes"

# Apply migrations
alembic upgrade head
```

## Vector Store Setup

The application will automatically create the Qdrant collection if it doesn't exist. Initial content is ingested via the `initialize_db.py` script.

## Security Considerations

1. **API Keys**: Store API keys securely and never commit them to version control
2. **Rate Limiting**: The application includes built-in rate limiting to prevent abuse
3. **CORS**: The application allows all origins by default; configure appropriately for production
4. **Environment Variables**: Use environment variables for all configuration

## Monitoring and Logging

The application logs to standard output with different levels:
- INFO: General operation information
- WARNING: Non-critical issues
- ERROR: Errors that may affect functionality

## Performance Tuning

1. **Rate Limiting**: Adjust `RATE_LIMIT_REQUESTS` and `RATE_LIMIT_WINDOW` based on your capacity
2. **Database Connection Pooling**: The application uses SQLAlchemy's connection pooling
3. **Context Window**: Adjust `MAX_CONTEXT_TOKENS` to balance response quality and cost
4. **Session Retention**: Configure `SESSION_RETENTION_DAYS` based on your storage capacity

## Scaling

### Horizontal Scaling
The application is stateless and can be scaled horizontally behind a load balancer.

### Database Scaling
- Neon Postgres offers serverless scaling options
- Consider read replicas for high-read workloads

### Vector Store Scaling
- Qdrant Cloud offers various scaling tiers
- Consider collection sharding for very large content sets

## Troubleshooting

### Common Issues
1. **Connection Errors**: Verify API keys and database/Vector store connection strings
2. **Rate Limiting**: Check logs for 429 responses and adjust rate limits
3. **Empty Responses**: Verify content has been properly ingested into the vector store

### Logs
Check application logs for error messages and performance information.

## Health Checks

The application provides a health check endpoint at `/health` that returns the application status.