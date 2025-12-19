"""
Authentication Services for the Physical AI Book Platform

This module provides business logic for user authentication and profile management,
including registration, login, profile updates, and validation.
"""

from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from typing import Optional, Dict, Any
from datetime import datetime, timedelta
import re
import bcrypt
import jwt
from .models import User, UserProfile
from database import get_db


class AuthService:
    """
    Service class for handling authentication-related operations.
    """

    @staticmethod
    def validate_email(email: str) -> bool:
        """
        Validate email format using regex.
        """
        pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.match(pattern, email) is not None

    @staticmethod
    def validate_password(password: str) -> bool:
        """
        Validate password strength: 8+ chars with uppercase, lowercase, and number.
        """
        if len(password) < 8:
            return False

        has_upper = any(c.isupper() for c in password)
        has_lower = any(c.islower() for c in password)
        has_digit = any(c.isdigit() for c in password)

        return has_upper and has_lower and has_digit

    @staticmethod
    def hash_password(password: str) -> str:
        """
        Hash a password using bcrypt.
        """
        salt = bcrypt.gensalt()
        return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')

    @staticmethod
    def verify_password(password: str, hashed: str) -> bool:
        """
        Verify a password against its hash.
        """
        return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))

    @staticmethod
    def validate_experience_level(level: str) -> bool:
        """
        Validate that experience level is one of: 'beginner', 'intermediate', 'advanced'.
        """
        return level in ['beginner', 'intermediate', 'advanced']

    @staticmethod
    def validate_background_preference(preference: str) -> bool:
        """
        Validate that background preference is one of: 'software', 'hardware', 'mixed'.
        """
        return preference in ['software', 'hardware', 'mixed']

    def register_user(
        self,
        db: Session,
        email: str,
        password: str,
        software_experience: str,
        hardware_experience: str,
        background_preference: str
    ) -> Dict[str, Any]:
        """
        Register a new user with profile information.

        Args:
            db: Database session
            email: User's email address
            password: User's password
            software_experience: User's software background level
            hardware_experience: User's hardware background level
            background_preference: User's preferred focus area

        Returns:
            Dictionary with success status and user data or error message
        """
        # Validate inputs
        if not self.validate_email(email):
            return {"success": False, "error": "Invalid email format"}

        if not self.validate_password(password):
            return {"success": False, "error": "Password must be 8+ characters with uppercase, lowercase, and number"}

        if not self.validate_experience_level(software_experience):
            return {"success": False, "error": "Invalid software experience level"}

        if not self.validate_experience_level(hardware_experience):
            return {"success": False, "error": "Invalid hardware experience level"}

        if not self.validate_background_preference(background_preference):
            return {"success": False, "error": "Invalid background preference"}

        try:
            # Check if user already exists
            existing_user = db.query(User).filter(User.email == email).first()
            if existing_user:
                return {"success": False, "error": "Email already registered"}

            # Hash the password
            hashed_password = self.hash_password(password)

            # Create user
            user = User(
                email=email,
                password_hash=hashed_password
            )
            db.add(user)
            db.flush()  # Get the user ID without committing

            # Create profile
            profile = UserProfile(
                user_id=user.id,
                software_experience=software_experience,
                hardware_experience=hardware_experience,
                background_preference=background_preference
            )
            db.add(profile)
            db.commit()
            db.refresh(user)

            return {
                "success": True,
                "user": {
                    "id": str(user.id),
                    "email": user.email,
                    "created_at": user.created_at.isoformat() if user.created_at else None
                },
                "profile": {
                    "software_experience": profile.software_experience,
                    "hardware_experience": profile.hardware_experience,
                    "background_preference": profile.background_preference
                }
            }

        except IntegrityError:
            db.rollback()
            return {"success": False, "error": "Email already registered"}
        except Exception as e:
            db.rollback()
            return {"success": False, "error": f"Registration failed: {str(e)}"}

    def authenticate_user(
        self,
        db: Session,
        email: str,
        password: str
    ) -> Dict[str, Any]:
        """
        Authenticate a user with email and password.

        Args:
            db: Database session
            email: User's email address
            password: User's password

        Returns:
            Dictionary with success status and user data or error message
        """
        try:
            user = db.query(User).filter(User.email == email, User.is_active == True).first()

            if not user or not self.verify_password(password, user.password_hash):
                return {"success": False, "error": "Invalid email or password"}

            # Get user profile
            profile = db.query(UserProfile).filter(UserProfile.user_id == user.id).first()

            return {
                "success": True,
                "user": {
                    "id": str(user.id),
                    "email": user.email,
                    "is_active": user.is_active,
                    "email_verified": user.email_verified,
                    "created_at": user.created_at.isoformat() if user.created_at else None
                },
                "profile": profile.to_dict() if profile else None
            }
        except Exception as e:
            return {"success": False, "error": f"Authentication failed: {str(e)}"}

    def get_user_profile(
        self,
        db: Session,
        user_id: str
    ) -> Dict[str, Any]:
        """
        Get a user's profile information.

        Args:
            db: Database session
            user_id: User's ID

        Returns:
            Dictionary with success status and profile data or error message
        """
        try:
            profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()

            if not profile:
                return {"success": False, "error": "Profile not found"}

            return {
                "success": True,
                "profile": profile.to_dict()
            }
        except Exception as e:
            return {"success": False, "error": f"Failed to get profile: {str(e)}"}

    def update_user_profile(
        self,
        db: Session,
        user_id: str,
        software_experience: Optional[str] = None,
        hardware_experience: Optional[str] = None,
        background_preference: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Update a user's profile information.

        Args:
            db: Database session
            user_id: User's ID
            software_experience: New software experience level (optional)
            hardware_experience: New hardware experience level (optional)
            background_preference: New background preference (optional)

        Returns:
            Dictionary with success status and updated profile or error message
        """
        try:
            profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()

            if not profile:
                return {"success": False, "error": "Profile not found"}

            # Update fields if provided and valid
            if software_experience is not None:
                if not self.validate_experience_level(software_experience):
                    return {"success": False, "error": "Invalid software experience level"}
                profile.software_experience = software_experience

            if hardware_experience is not None:
                if not self.validate_experience_level(hardware_experience):
                    return {"success": False, "error": "Invalid hardware experience level"}
                profile.hardware_experience = hardware_experience

            if background_preference is not None:
                if not self.validate_background_preference(background_preference):
                    return {"success": False, "error": "Invalid background preference"}
                profile.background_preference = background_preference

            profile.updated_at = datetime.utcnow()
            db.commit()
            db.refresh(profile)

            return {
                "success": True,
                "profile": profile.to_dict()
            }
        except Exception as e:
            db.rollback()
            return {"success": False, "error": f"Failed to update profile: {str(e)}"}

    def create_session_token(self, user_id: str) -> str:
        """
        Create a session token for the user (simplified implementation).
        In a real system, you'd use JWT or similar with proper security.
        """
        import secrets
        import time
        import os

        # For this implementation, we'll use a simple approach
        # In production, use proper JWT with signing
        payload = {
            "user_id": user_id,
            "exp": datetime.utcnow() + timedelta(days=7),  # 7-day timeout
            "iat": datetime.utcnow()
        }

        # Use a secret key from environment
        from os import environ
        secret = environ.get("AUTH_SECRET", "dev-secret-key-change-in-production")

        return jwt.encode(payload, secret, algorithm="HS256")

    def verify_session_token(self, token: str) -> Optional[str]:
        """
        Verify a session token and return user_id if valid.
        """
        try:
            from os import environ
            secret = environ.get("AUTH_SECRET", "dev-secret-key-change-in-production")

            payload = jwt.decode(token, secret, algorithms=["HS256"])
            user_id = payload.get("user_id")

            # Check if token is still valid
            exp = payload.get("exp")
            if exp and datetime.utcnow().timestamp() > exp:
                return None

            return user_id
        except jwt.ExpiredSignatureError:
            return None
        except jwt.InvalidTokenError:
            return None
        except Exception:
            return None


# Create a global instance
auth_service = AuthService()