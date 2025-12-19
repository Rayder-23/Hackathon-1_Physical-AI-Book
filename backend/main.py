from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from typing import List, Optional, Dict
from sqlalchemy.orm import Session
import uuid
import logging
import time
from datetime import datetime, timedelta
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import local modules
from database import SessionLocal, get_db
from models import ChatSession, UserQuery, AgentResponse, RetrievedContent
from agent import rag_agent, QueryMode
from vector_store import vector_store
from rate_limiting import rate_limiter
from session_service import session_service
from src.auth.routes import router as auth_router

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Physical AI Book RAG Chatbot API",
    description="API for the Physical AI Book RAG Chatbot with OpenAI Agents SDK integration",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for request/response
class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    mode: str = "full-book"  # "full-book" or "selected-text"
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    id: str
    session_id: str
    message: str
    citations: List[Dict]
    timestamp: datetime

class RetrieveRequest(BaseModel):
    query: str
    mode: str = "full-book"
    selected_text: Optional[str] = None
    limit: int = 5

class RetrieveResponse(BaseModel):
    results: List[Dict]

class SessionResponse(BaseModel):
    id: str
    user_id: Optional[str]
    mode: str
    created_at: datetime
    updated_at: datetime

# Dependency to get database session
def get_database():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@app.on_event("startup")
def startup_event():
    """
    Startup event to initialize the database
    """
    from database import create_tables
    create_tables()
    logger.info("Database tables created successfully")

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: Request, chat_request: ChatRequest, db: Session = Depends(get_database)):
    """
    Handle chat requests and return AI-generated responses
    """
    # Get client IP for rate limiting
    client_ip = request.client.host

    # Check rate limit
    if not rate_limiter.is_allowed(client_ip):
        reset_time = rate_limiter.get_reset_time(client_ip)
        if reset_time:
            retry_after = int(reset_time - time.time())
            return JSONResponse(
                status_code=429,
                content={"detail": f"Rate limit exceeded. Try again in {retry_after} seconds."},
                headers={"Retry-After": str(retry_after)}
            )
        else:
            return JSONResponse(
                status_code=429,
                content={"detail": "Rate limit exceeded. Please try again later."}
            )

    try:
        # Validate mode
        if chat_request.mode not in ["full-book", "selected-text"]:
            raise HTTPException(status_code=400, detail="Invalid mode. Use 'full-book' or 'selected-text'")

        # Validate selected-text mode requirements
        if chat_request.mode == "selected-text" and not chat_request.selected_text:
            raise HTTPException(status_code=400, detail="selected_text is required for selected-text mode")

        # Get or create session using session service
        if chat_request.session_id:
            session = session_service.get_session(db, chat_request.session_id)
            if not session:
                # Create new session with the provided ID
                session = session_service.create_session(
                    db=db,
                    mode=chat_request.mode
                )
        else:
            # Create new session
            session = session_service.create_session(
                db=db,
                mode=chat_request.mode
            )

        # Update session timestamp
        session = session_service.update_session_timestamp(db, session)

        # Validate the query mode
        query_mode = QueryMode(chat_request.mode)

        # Process the query through the RAG agent
        agent_response = rag_agent.process_query(
            user_query=chat_request.message,
            mode=query_mode,
            selected_text=chat_request.selected_text
        )

        # Check if the agent returned an error
        if "Error" in agent_response.content or "error" in agent_response.content.lower():
            raise HTTPException(status_code=500, detail=agent_response.content)

        # Create UserQuery record
        user_query = UserQuery(
            session_id=session.id,
            content=chat_request.message,
            selected_text=chat_request.selected_text
        )
        db.add(user_query)
        db.commit()
        db.refresh(user_query)  # Refresh to get the generated ID

        # Create AgentResponse record
        response_record = AgentResponse(
            query_id=user_query.id,
            content=agent_response.content,
            citations=agent_response.citations,
            token_usage=agent_response.token_usage
        )
        db.add(response_record)
        db.commit()

        # Create RetrievedContent records if citations exist
        for citation in agent_response.citations:
            if citation.get("id"):
                retrieved_content = RetrievedContent(
                    query_id=user_query.id,
                    book_content_id=uuid.UUID(citation["id"]),
                    relevance_score=citation.get("relevance_score", 0.0),
                    context_window=citation.get("content", "")[:500]  # Limit length
                )
                db.add(retrieved_content)

        db.commit()

        # Update session timestamp
        session.updated_at = datetime.utcnow()
        db.commit()

        # Prepare response
        response = ChatResponse(
            id=str(response_record.id),
            session_id=str(session.id),
            message=agent_response.content,
            citations=agent_response.citations,
            timestamp=datetime.utcnow()
        )

        logger.info(f"Chat response generated for session {session.id}")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.post("/api/retrieve", response_model=RetrieveResponse)
async def retrieve_endpoint(retrieve_request: RetrieveRequest):
    """
    Perform content retrieval from the vector store
    """
    try:
        from embeddings import embeddings_service

        # Generate embedding for the query
        query_embedding = embeddings_service.embed_text(retrieve_request.query)

        # Perform search based on mode
        if retrieve_request.mode == "selected-text" and retrieve_request.selected_text:
            results = vector_store.search_selected_text(
                query_vector=query_embedding,
                selected_text=retrieve_request.selected_text,
                limit=retrieve_request.limit
            )
        else:
            results = vector_store.search_content(
                query_vector=query_embedding,
                limit=retrieve_request.limit,
                mode=retrieve_request.mode
            )

        return RetrieveResponse(results=results)

    except Exception as e:
        logger.error(f"Error in retrieve endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.get("/api/sessions/{session_id}", response_model=SessionResponse)
async def get_session(session_id: str, db: Session = Depends(get_database)):
    """
    Get session details by ID
    """
    try:
        session_uuid = uuid.UUID(session_id)
        session = db.query(ChatSession).filter(ChatSession.id == session_uuid).first()

        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        return SessionResponse(
            id=str(session.id),
            user_id=str(session.user_id) if session.user_id else None,
            mode=session.mode,
            created_at=session.created_at,
            updated_at=session.updated_at
        )
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid session ID format")
    except Exception as e:
        logger.error(f"Error in get session endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "timestamp": datetime.utcnow()}

# Include auth routes
app.include_router(auth_router, prefix="/api", tags=["authentication"])

@app.get("/")
async def root():
    """
    Root endpoint
    """
    return {
        "message": "Physical AI Book RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": [
            "/api/chat - Chat endpoint",
            "/api/retrieve - Content retrieval",
            "/api/sessions/{session_id} - Get session details",
            "/api/auth - Authentication endpoints",
            "/health - Health check"
        ]
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)