from sqlalchemy import create_engine, Column, String, DateTime, Text, Float, Integer, ForeignKey, JSON
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

    # Relationships
    queries = relationship("UserQuery", back_populates="session")

    def __repr__(self):
        return f"<ChatSession(id={self.id}, mode={self.mode})>"

class UserQuery(Base):
    __tablename__ = 'user_queries'

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey('chat_sessions.id'), nullable=False)
    content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    selected_text = Column(Text, nullable=True)  # For selected-text mode
    query_embedding = Column(String)  # Store as string representation of vector

    # Relationships
    session = relationship("ChatSession", back_populates="queries")
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