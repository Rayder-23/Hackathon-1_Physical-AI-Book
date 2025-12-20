from sqlalchemy.orm import Session
from models.models import UserSession
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

    def create_session(self, db: Session, mode: str = "full-book", user_id: Optional[str] = None, chatkit_session_id: Optional[str] = None) -> UserSession:
        """
        Create a new chat session

        Args:
            db: Database session
            mode: Query mode ('full-book' or 'selected-text') - stored in metadata for compatibility
            user_id: User ID for authenticated sessions (None for anonymous)
            chatkit_session_id: Chatkit session identifier (optional)

        Returns:
            Created UserSession object
        """
        # Generate a chatkit session ID if not provided
        if not chatkit_session_id:
            chatkit_session_id = f"session_{str(uuid.uuid4())}"

        # Determine expiration time (24 hours for guest sessions)
        expires_at = None
        if user_id is None:  # Guest session
            expires_at = datetime.utcnow() + timedelta(hours=24)

        session = UserSession(
            user_id=user_id,
            chatkit_session_id=chatkit_session_id,
            expires_at=expires_at
        )

        db.add(session)
        db.commit()
        db.refresh(session)

        logger.info(f"Created new session {session.id} for {'user ' + str(user_id) if user_id else 'anonymous user'}")
        return session

    def get_session(self, db: Session, session_id: str) -> Optional[UserSession]:
        """
        Get a session by ID

        Args:
            db: Database session
            session_id: Session UUID as string

        Returns:
            UserSession object or None if not found
        """
        try:
            session_uuid = uuid.UUID(session_id)
            return db.query(UserSession).filter(UserSession.id == session_uuid).first()
        except ValueError:
            logger.error(f"Invalid session ID format: {session_id}")
            return None

    def get_session_by_chatkit_id(self, db: Session, chatkit_session_id: str) -> Optional[UserSession]:
        """
        Retrieve a session by its Chatkit session ID.

        Args:
            db: Database session
            chatkit_session_id: Chatkit session identifier

        Returns:
            UserSession: The session object or None if not found
        """
        return db.query(UserSession).filter(
            UserSession.chatkit_session_id == chatkit_session_id
        ).first()

    def update_session_timestamp(self, db: Session, session: UserSession) -> UserSession:
        """
        Update the session's last updated timestamp

        Args:
            db: Database session
            session: UserSession object to update

        Returns:
            Updated UserSession object
        """
        session.last_activity_at = datetime.utcnow()

        # Extend guest session expiration if needed
        if session.user_id is None and session.expires_at:  # Guest session
            session.expires_at = datetime.utcnow() + timedelta(hours=24)

        db.commit()
        db.refresh(session)
        return session

    def update_session_activity(self, db: Session, session_id: str):
        """
        Update the last activity timestamp for a session.

        Args:
            db: Database session
            session_id: Session UUID
        """
        session = db.query(UserSession).filter(
            UserSession.id == session_id
        ).first()

        if session:
            session.last_activity_at = datetime.utcnow()

            # Extend guest session expiration if needed
            if session.user_id is None and session.expires_at:  # Guest session
                session.expires_at = datetime.utcnow() + timedelta(hours=24)

            db.commit()

    def is_permanent_session(self, session: UserSession) -> bool:
        """
        Check if a session is permanent (authenticated) or temporary (anonymous)

        Args:
            session: UserSession object

        Returns:
            True if session is permanent, False if temporary
        """
        return session.user_id is not None

    def should_cleanup_session(self, session: UserSession) -> bool:
        """
        Determine if a session should be cleaned up based on retention policy

        Args:
            session: UserSession object

        Returns:
            True if session should be cleaned up, False otherwise
        """
        if self.is_permanent_session(session):
            # Permanent sessions (authenticated) are not cleaned up automatically
            return False
        else:
            # Temporary sessions (anonymous) are cleaned up after retention period
            cutoff_date = datetime.utcnow() - timedelta(days=self.retention_days)
            if session.expires_at:
                return session.expires_at < datetime.utcnow()
            return False

    def cleanup_expired_sessions(self, db: Session) -> int:
        """
        Clean up expired temporary sessions

        Args:
            db: Database session

        Returns:
            Number of sessions cleaned up
        """
        current_time = datetime.utcnow()

        expired_sessions = db.query(UserSession).filter(
            UserSession.user_id.is_(None),  # Only anonymous sessions
            UserSession.expires_at < current_time,  # That have expired
            UserSession.is_active == True  # That are still active
        ).all()

        count = 0
        for session in expired_sessions:
            session.is_active = False  # Mark as inactive instead of deleting to maintain referential integrity
            count += 1

        if count > 0:
            db.commit()
            logger.info(f"Cleaned up {count} expired temporary sessions")

        return count

# Create a global instance
session_service = SessionService()