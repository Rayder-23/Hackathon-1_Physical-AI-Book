from sqlalchemy.orm import Session
from models import ChatSession
from datetime import datetime, timedelta
import uuid
import os
from typing import Optional
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SessionService:
    def __init__(self):
        self.retention_days = int(os.getenv("SESSION_RETENTION_DAYS", 30))

    def create_session(self, db: Session, mode: str, user_id: Optional[str] = None) -> ChatSession:
        """
        Create a new chat session

        Args:
            db: Database session
            mode: Query mode ('full-book' or 'selected-text')
            user_id: User ID for authenticated sessions (None for anonymous)

        Returns:
            Created ChatSession object
        """
        session = ChatSession(
            mode=mode,
            user_id=uuid.UUID(user_id) if user_id else None
        )

        db.add(session)
        db.commit()
        db.refresh(session)

        logger.info(f"Created new session {session.id} for {'user ' + str(user_id) if user_id else 'anonymous user'}")
        return session

    def get_session(self, db: Session, session_id: str) -> Optional[ChatSession]:
        """
        Get a session by ID

        Args:
            db: Database session
            session_id: Session UUID as string

        Returns:
            ChatSession object or None if not found
        """
        try:
            session_uuid = uuid.UUID(session_id)
            return db.query(ChatSession).filter(ChatSession.id == session_uuid).first()
        except ValueError:
            logger.error(f"Invalid session ID format: {session_id}")
            return None

    def update_session_timestamp(self, db: Session, session: ChatSession) -> ChatSession:
        """
        Update the session's last updated timestamp

        Args:
            db: Database session
            session: ChatSession object to update

        Returns:
            Updated ChatSession object
        """
        session.updated_at = datetime.utcnow()
        db.commit()
        db.refresh(session)
        return session

    def is_permanent_session(self, session: ChatSession) -> bool:
        """
        Check if a session is permanent (authenticated) or temporary (anonymous)

        Args:
            session: ChatSession object

        Returns:
            True if session is permanent, False if temporary
        """
        return session.user_id is not None

    def should_cleanup_session(self, session: ChatSession) -> bool:
        """
        Determine if a session should be cleaned up based on retention policy

        Args:
            session: ChatSession object

        Returns:
            True if session should be cleaned up, False otherwise
        """
        if self.is_permanent_session(session):
            # Permanent sessions (authenticated) are not cleaned up automatically
            return False
        else:
            # Temporary sessions (anonymous) are cleaned up after retention period
            cutoff_date = datetime.utcnow() - timedelta(days=self.retention_days)
            return session.created_at < cutoff_date

    def cleanup_expired_sessions(self, db: Session) -> int:
        """
        Clean up expired temporary sessions

        Args:
            db: Database session

        Returns:
            Number of sessions cleaned up
        """
        cutoff_date = datetime.utcnow() - timedelta(days=self.retention_days)

        expired_sessions = db.query(ChatSession).filter(
            ChatSession.user_id.is_(None),  # Only anonymous sessions
            ChatSession.created_at < cutoff_date
        ).all()

        count = 0
        for session in expired_sessions:
            # In a full implementation, you'd also need to delete related queries,
            # responses, and retrieved content to maintain referential integrity
            db.delete(session)
            count += 1

        if count > 0:
            db.commit()
            logger.info(f"Cleaned up {count} expired temporary sessions")

        return count

# Create a global instance
session_service = SessionService()