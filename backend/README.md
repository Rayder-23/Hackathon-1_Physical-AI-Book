# Physical AI Book RAG Chatbot Backend

This is the backend service for the Physical AI Book RAG Chatbot, built with FastAPI and using OpenAI Agents SDK for orchestration.

## Architecture

The backend consists of:
- FastAPI application serving the API endpoints
- OpenAI Agent for RAG orchestration
- Vector store integration with Qdrant Cloud
- Database integration with Neon Postgres
- Qwen embeddings for content vectorization

## Components

### Database Models
- `ChatSession`: Tracks user chat sessions
- `UserQuery`: Stores user questions
- `AgentResponse`: Stores AI-generated responses
- `RetrievedContent`: Links queries to relevant book content
- `BookContent`: Stores the book content with embeddings

### Services
- `vector_store.py`: Manages vector storage with Qdrant
- `embeddings.py`: Handles text embedding generation with Qwen models
- `document_ingestion.py`: Processes and indexes book content
- `agent.py`: Implements RAG agent with OpenAI Agents SDK
- `database.py`: Handles database connections and sessions

## Environment Variables

Create a `.env` file in the backend directory with the following variables:

```bash
# OpenRouter API Configuration
OPENROUTER_API_KEY=your_openrouter_api_key_here
OPENROUTER_MODEL_NAME=your_preferred_model_here  # e.g., "openai/gpt-4-turbo"

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
RATE_LIMIT_WINDOW=3600  # in seconds
```

## API Endpoints

- `POST /api/chat` - Process chat queries with RAG
- `POST /api/retrieve` - Perform content retrieval
- `GET /api/sessions/{session_id}` - Get session details
- `GET /health` - Health check

## Running the Application

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables in `.env`

3. Run the application:
```bash
python main.py
```

Or with uvicorn:
```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

## Endpoints

### Chat Endpoint
```bash
POST /api/chat
Content-Type: application/json

{
  "message": "Your question here",
  "session_id": "optional session ID",
  "mode": "full-book",  // or "selected-text"
  "selected_text": "optional selected text for selected-text mode"
}
```

### Content Retrieval
```bash
POST /api/retrieve
Content-Type: application/json

{
  "query": "Your search query",
  "mode": "full-book",  // or "selected-text"
  "selected_text": "optional selected text",
  "limit": 5
}
```

## Backend Structure

This section describes the organized structure of the backend codebase after restructure.

### Current Structure Analysis

The backend structure contains a mix of core application files, API routes, data models, and utility functions organized in logical directories.

### Recommended Structure

```
backend/
├── main.py                           # Application entry point and FastAPI app configuration
├── __init__.py                       # Package initialization (enables imports)
├── alembic.ini                       # Alembic configuration for database migrations
├── README.md                         # Project documentation and setup instructions
├── requirements.txt                  # Python package dependencies
├── agents/                           # AI/ML agents and processing
│   ├── __init__.py                   # Package initialization
│   └── agent.py                      # RAG agent implementation using OpenAI API (from original agent.py)
├── auth/                             # Authentication module
│   ├── __init__.py                   # Package initialization
│   ├── models.py                     # Authentication-related models (from src/auth/)
│   ├── routes.py                     # Authentication API routes (from src/auth/)
│   └── services.py                   # Authentication services (from src/auth/)
├── data/                             # Data handling and storage utilities
│   ├── __init__.py                   # Package initialization
│   ├── embeddings.py                 # Embedding generation and management
│   ├── vector_store.py               # Vector store operations
│   ├── document_ingestion.py         # Document processing and ingestion
│   └── validation.py                 # Data validation utilities
├── database/                         # Database operations and management
│   ├── __init__.py                   # Package initialization
│   ├── database.py                   # Database connection management (from original database.py)
│   └── migrations/                   # Alembic migration files
│       ├── __init__.py               # Package initialization
│       ├── env.py                    # Migration environment configuration
│       ├── script.py.mako            # Alembic migration script template
│       └── versions/                 # Individual migration files
├── models/                           # Database models and schema definitions
│   ├── __init__.py                   # Package initialization
│   └── models.py                     # Database models and schema definitions (from original models.py)
├── services/                         # Business logic and service layer
│   ├── __init__.py                   # Package initialization
│   └── session_service.py            # Session management service (from original session_service.py)
├── utils/                            # Utility functions and helpers
│   ├── __init__.py                   # Package initialization
│   └── rate_limiting.py              # Rate limiting utilities (from original rate_limiting.py)
├── chatkit/                          # Chatkit integration components (standalone)
│   ├── __init__.py                   # Package initialization
│   ├── adapter.py                    # Adapter to interface between Chatkit and RAG system
│   └── routes.py                     # Chatkit-specific API routes
├── scripts/                          # Utility scripts
│   ├── __init__.py                   # Package initialization
│   ├── initialize_db.py              # Database initialization script (from original initialize_db.py)
│   └── reset_qdrant.py               # Vector store reset utility (from original reset_qdrant.py)
└── tests/                            # Test files
    ├── __init__.py                   # Package initialization
    └── test_basic_functionality.py   # Basic functionality tests (from original test_basic_functionality.py)
```

### File Descriptions

#### Core Files
- **main.py**: Application entry point that initializes FastAPI app, sets up middleware, and includes all API routes
- **__init__.py**: Package initialization file that makes Python treat directories as packages
- **alembic.ini**: Configuration file for Alembic database migration tool
- **requirements.txt**: List of Python package dependencies for the project

#### AI/ML Agents
- **agents/agent.py**: Main RAG agent implementation using OpenAI API (from original agent.py)

#### Authentication
- **auth/models.py**: Authentication-related models (from src/auth/models.py)
- **auth/routes.py**: Authentication API routes (from src/auth/routes.py)
- **auth/services.py**: Authentication services (from src/auth/services.py)

#### Data Layer
- **data/embeddings.py**: Embedding generation and management services
- **data/vector_store.py**: Vector store operations and search functionality
- **data/document_ingestion.py**: Document processing and ingestion pipeline
- **data/validation.py**: Data validation utilities

#### Database Layer
- **database/database.py**: Database connection and session management (from original database.py)
- **database/migrations/env.py**: Migration environment configuration
- **database/migrations/script.py.mako**: Alembic migration script template
- **database/migrations/versions/**: Individual migration files for schema evolution

#### Models Layer
- **models/models.py**: Database models and schema definitions (from original models.py)

#### Services Layer
- **services/session_service.py**: Session management service (from original session_service.py)

#### Utilities
- **utils/rate_limiting.py**: Rate limiting utilities (from original rate_limiting.py)

#### Chatkit Integration (Standalone)
- **chatkit/adapter.py**: Adapter to interface between Chatkit and the RAG system
- **chatkit/routes.py**: Chatkit-specific API routes

#### Scripts
- **scripts/initialize_db.py**: Database initialization script (from original initialize_db.py)
- **scripts/reset_qdrant.py**: Vector store reset utility (from original reset_qdrant.py)

#### Tests
- **tests/test_basic_functionality.py**: Basic functionality tests (from original test_basic_functionality.py)

### Purpose of `__init__.py` Files

The `__init__.py` files serve several important purposes in Python package structure:

1. **Package Recognition**: They tell Python that a directory should be treated as a package, allowing for imports from that directory.
2. **Namespace Control**: They can control what gets imported when someone imports the package using `from package import *`.
3. **Initialization Code**: They can contain initialization code that runs when the package is first imported.
4. **Relative Imports**: They enable relative imports between modules within the same package.
5. **Clean Imports**: They allow for cleaner import statements by defining what should be accessible at the package level.
6. **Package-level Attributes**: They can define package-level attributes and documentation.

### Benefits of the New Structure

1. **Separation of Concerns**: Each layer has a specific responsibility, making the codebase more maintainable
2. **Scalability**: New features can be added in the appropriate layer without disrupting existing functionality
3. **Testability**: Each component can be tested independently
4. **Readability**: Clear organization makes it easier to understand and navigate the codebase
5. **Best Practices**: Follows common Python and FastAPI project organization patterns
6. **Maintainability**: Changes to one layer don't unnecessarily affect other layers