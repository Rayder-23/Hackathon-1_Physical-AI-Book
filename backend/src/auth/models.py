"""
Authentication and User Profile Models for the Physical AI Book Platform

This module defines the data models for user authentication and profile management,
including user accounts and background information for personalization.
"""

from sqlalchemy import Column, Integer, String, Boolean, DateTime, UUID, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.sql import func
from database import Base
import uuid


class User(Base):
    """
    User model representing a registered user account with authentication credentials.
    This model works with better-auth for authentication while maintaining
    compatibility with the existing system.
    """
    __tablename__ = "users"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())
    is_active = Column(Boolean, default=True)
    email_verified = Column(Boolean, default=False)

    def __repr__(self):
        return f"<User(id={self.id}, email='{self.email}')>"


class UserProfile(Base):
    """
    UserProfile model containing user background information for content personalization.
    This model stores software and hardware experience levels that are required at
    registration and can be updated later.
    """
    __tablename__ = "user_profiles"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(PostgresUUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    software_experience = Column(String(20), nullable=False)  # beginner, intermediate, advanced
    hardware_experience = Column(String(20), nullable=False)  # beginner, intermediate, advanced
    background_preference = Column(String(20), nullable=False)  # software, hardware, mixed
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

    def __repr__(self):
        return f"<UserProfile(id={self.id}, user_id={self.user_id})>"

    def to_dict(self):
        """
        Convert the profile to a dictionary representation for API responses.
        """
        return {
            "id": str(self.id),
            "user_id": str(self.user_id),
            "software_experience": self.software_experience,
            "hardware_experience": self.hardware_experience,
            "background_preference": self.background_preference,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }