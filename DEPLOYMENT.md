# Railway Deployment Configuration

This directory contains the necessary files to deploy the Physical AI Book RAG Chatbot backend to Railway.

## Files Included

- `railway.json` - Railway configuration file with environment variables and deployment settings
- `Dockerfile` - Container configuration for the application
- `Procfile` - Process type definitions for Railway
- `requirements.txt` - Python dependencies (copied from backend/)
- `test_deployment.py` - Test script to verify deployment configuration

## Deployment Steps

1. **Install Railway CLI** (optional, for local deployment):
   ```bash
   npm install -g @railway/cli
   ```

2. **Connect to Railway**:
   - If using CLI: `railway login`
   - Or link your project: `railway link`
   - Or deploy directly from GitHub through Railway dashboard

3. **Set Environment Variables**:
   The following environment variables are required for the application to work:

   - `OPENROUTER_API_KEY` - Your OpenRouter API key for AI model access
   - `QDRANT_URL` - Your Qdrant Cloud URL for vector storage
   - `QDRANT_API_KEY` - Your Qdrant API key
   - `DATABASE_URL` - Your Neon Postgres database connection string

   Optional variables with defaults:
   - `OPENROUTER_MODEL_NAME` - Model name for OpenRouter API (default: openai/gpt-4o-mini)
   - `QDRANT_COLLECTION_NAME` - Qdrant collection name (default: book_content)
   - `MAX_CONTEXT_TOKENS` - Maximum context tokens (default: 2000)
   - `SESSION_RETENTION_DAYS` - Days to retain sessions (default: 30)
   - `RATE_LIMIT_REQUESTS` - Number of requests allowed per time window (default: 100)
   - `RATE_LIMIT_WINDOW` - Time window for rate limiting in seconds (default: 3600)
   - `CIRCUIT_BREAKER_FAILURE_THRESHOLD` - Circuit breaker failure threshold (default: 5)
   - `CIRCUIT_BREAKER_RECOVERY_TIMEOUT` - Circuit breaker recovery timeout (default: 60)

4. **Deploy**:
   - Using CLI: `railway up` or `railway deploy`
   - Or deploy through Railway dashboard after connecting your GitHub repository

## Important Note on PORT Configuration

Railway automatically provides a `PORT` environment variable. The application is configured to work with Railway's port management:

- The `railway.json` file specifies the start command as: `uvicorn main:app --host 0.0.0.0 --port $PORT`
- The application will automatically use the port provided by Railway
- No need to manually set the `PORT` environment variable in Railway
- The default `PORT` value in railway.json (8000) is just a fallback and will be overridden by Railway

## Health Check

The application includes a health check endpoint at `/health` that returns:
```json
{
  "status": "healthy",
  "timestamp": "2024-12-20T00:00:00.000000"
}
```

## API Endpoints

Once deployed, the following endpoints will be available:

- `POST /api/chat` - Process chat queries with RAG
- `POST /api/retrieve` - Perform content retrieval
- `GET /api/sessions/{session_id}` - Get session details
- `GET /health` - Health check
- `GET /` - Root endpoint with API documentation

## Troubleshooting

- If deployment fails, check that all required environment variables are set
- Verify that your Qdrant and database connections are properly configured
- Check the Railway logs for specific error messages
- The application uses a circuit breaker pattern that may temporarily block requests if there are too many failures

## Scaling

The application is designed to be stateless and can be scaled horizontally. Session data is stored in the database, so multiple instances can share the same session state.