from sqlalchemy import create_engine, Column, String, DateTime, Text, Float, Integer, ForeignKey, JSON, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

Base = declarative_base()

class ChatSession(Base):
    __tablename__ = 'chat_sessions'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), nullable=True)  # nullable for anonymous users
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    mode = Column(String(20), default='full-book')  # 'full-book' or 'selected-text'
    metadata_ = Column('metadata', JSON, default={})

    def __repr__(self):
        return f"<ChatSession(id={self.id}, mode={self.mode})>"

class UserQuery(Base):
    __tablename__ = 'user_queries'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey('sessions.id'), nullable=False)
    content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    selected_text = Column(Text, nullable=True)  # For selected-text mode
    query_embedding = Column(String)  # Store as string representation of vector

    # Relationships
    session = relationship("UserSession", back_populates="queries")
    retrieved_content = relationship("RetrievedContent", back_populates="query")
    response = relationship("AgentResponse", uselist=False, back_populates="query")

    def __repr__(self):
        return f"<UserQuery(id={self.id}, content='{self.content[:50]}...')>"

class RetrievedContent(Base):
    __tablename__ = 'retrieved_content'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey('user_queries.id'), nullable=False)
    book_content_id = Column(UUID(as_uuid=True), ForeignKey('book_content.id'), nullable=False)
    relevance_score = Column(Float, nullable=False)  # Between 0 and 1
    retrieved_at = Column(DateTime, default=datetime.utcnow)
    context_window = Column(Text)  # Surrounding context of the retrieved content

    # Relationships
    query = relationship("UserQuery", back_populates="retrieved_content")
    book_content = relationship("BookContent", back_populates="retrieved_queries")

    def __repr__(self):
        return f"<RetrievedContent(id={self.id}, score={self.relevance_score})>"

class AgentResponse(Base):
    __tablename__ = 'agent_responses'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey('user_queries.id'), nullable=False)
    content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    citations = Column(JSON, default=[])  # Array of book content IDs
    token_usage = Column(JSON, default={})  # Input/output token counts

    # Relationships
    query = relationship("UserQuery", back_populates="response")

    def __repr__(self):
        return f"<AgentResponse(id={self.id}, content='{self.content[:50]}...')>"

class BookContent(Base):
    __tablename__ = 'book_content'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    section_title = Column(String(500), nullable=False)
    content = Column(Text, nullable=False)
    content_embedding = Column(String)  # Store as string representation of vector
    page_number = Column(Integer)
    chapter = Column(String(200))
    section = Column(String(200))
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationships
    retrieved_queries = relationship("RetrievedContent", back_populates="book_content")

    def __repr__(self):
        return f"<BookContent(id={self.id}, title='{self.section_title}')>"


# New models for Chatkit integration

class UserSession(Base):
    __tablename__ = 'sessions'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String(255), nullable=True)  # Reference to authenticated user (null for guest sessions)
    chatkit_session_id = Column(String(255), nullable=False)  # Chatkit session identifier
    created_at = Column(DateTime, default=datetime.utcnow)
    last_activity_at = Column(DateTime, default=datetime.utcnow)
    expires_at = Column(DateTime, nullable=True)  # Expiration time (for guest sessions)
    is_active = Column(Boolean, default=True)

    # Relationships
    queries = relationship("UserQuery", back_populates="session")
    conversations = relationship("Conversation", back_populates="session")
    rag_queries = relationship("RagQuery", back_populates="session")

    def __repr__(self):
        return f"<UserSession(id={self.id}, user_id={self.user_id}, chatkit_session_id={self.chatkit_session_id})>"


class Conversation(Base):
    __tablename__ = 'conversations'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chatkit_room_id = Column(String(255), nullable=False)  # Chatkit room identifier
    session_id = Column(UUID(as_uuid=True), ForeignKey('sessions.id'), nullable=False)  # Reference to user session
    title = Column(String(500), nullable=True)  # Auto-generated from first query
    created_at = Column(DateTime, default=datetime.utcnow)
    last_message_at = Column(DateTime, default=datetime.utcnow)
    message_count = Column(Integer, default=0)  # Number of messages in conversation
    is_active = Column(Boolean, default=True)

    # Relationships
    session = relationship("UserSession", back_populates="conversations")
    messages = relationship("ChatMessage", back_populates="conversation")

    def __repr__(self):
        return f"<Conversation(id={self.id}, chatkit_room_id={self.chatkit_room_id}, title={self.title})>"


class ChatMessage(Base):
    __tablename__ = 'chat_messages'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    room_id = Column(String(255), nullable=False)  # Chatkit room identifier
    sender_id = Column(String(255), nullable=False)  # User or system identifier
    sender_type = Column(String(20), nullable=False)  # 'user', 'bot', 'system'
    content = Column(Text, nullable=False)  # The message content
    timestamp = Column(DateTime, default=datetime.utcnow)
    message_type = Column(String(20), nullable=False)  # 'text', 'command', 'response'
    related_query_id = Column(UUID(as_uuid=True), nullable=True)  # Reference to related query in RAG system
    conversation_id = Column(UUID(as_uuid=True), ForeignKey('conversations.id'), nullable=False)  # Reference to conversation

    # Relationship
    conversation = relationship("Conversation", back_populates="messages")

    def __repr__(self):
        return f"<ChatMessage(id={self.id}, room_id={self.room_id}, sender_id={self.sender_id}, sender_type={self.sender_type})>"


class RagQuery(Base):
    __tablename__ = 'rag_queries'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey('sessions.id'), nullable=False)  # Reference to user session
    query_text = Column(Text, nullable=False)  # Original user query
    query_vector = Column(String, nullable=True)  # Vector representation of query (for Qdrant)
    context_documents = Column(JSON, nullable=True)  # Relevant documents retrieved from vector store
    response_text = Column(Text, nullable=True)  # Generated response to the query
    timestamp = Column(DateTime, default=datetime.utcnow)  # When query was processed
    response_time_ms = Column(Integer, nullable=True)  # Time taken to process the query
    relevance_score = Column(Float, nullable=True)  # Score of how relevant the response was

    # Relationship
    session = relationship("UserSession", back_populates="rag_queries")

    def __repr__(self):
        return f"<RagQuery(id={self.id}, session_id={self.session_id}, query_text='{self.query_text[:50]}...')>"