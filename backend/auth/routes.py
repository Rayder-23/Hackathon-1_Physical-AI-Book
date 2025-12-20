"""
Authentication Routes for the Physical AI Book Platform

This module defines the API endpoints for user authentication and profile management,
including registration, login, logout, and profile operations.
"""

from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from typing import Optional
from database.database import get_db
from .models import User
from .services import auth_service
from pydantic import BaseModel
import json


router = APIRouter(prefix="/auth", tags=["authentication"])

# Define request/response models
class RegisterRequest(BaseModel):
    email: str
    password: str
    software_experience: str
    hardware_experience: str
    background_preference: str

class LoginRequest(BaseModel):
    email: str
    password: str

class UpdateProfileRequest(BaseModel):
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    background_preference: Optional[str] = None


# HTTP Bearer token for authentication
security = HTTPBearer()


def get_current_user_id(credentials: HTTPAuthorizationCredentials = Depends(security), db: Session = Depends(get_db)):
    """
    Get the current user ID from the authorization token.
    """
    token = credentials.credentials
    user_id = auth_service.verify_session_token(token)

    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Verify user still exists in database
    user = db.query(User).filter(User.id == user_id, User.is_active == True).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User no longer exists",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user_id


@router.post("/register", status_code=status.HTTP_201_CREATED)
async def register(request: RegisterRequest, db: Session = Depends(get_db)):
    """
    Register a new user with profile information.

    Creates a new user account and associated profile with background information.
    """
    result = auth_service.register_user(
        db=db,
        email=request.email,
        password=request.password,
        software_experience=request.software_experience,
        hardware_experience=request.hardware_experience,
        background_preference=request.background_preference
    )

    if not result["success"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=result["error"]
        )

    # Create session token for auto-login
    token = auth_service.create_session_token(result["user"]["id"])

    return {
        "success": True,
        "message": "User registered successfully",
        "user": result["user"],
        "profile": result["profile"],
        "token": token
    }


@router.post("/login")
async def login(request: LoginRequest, db: Session = Depends(get_db)):
    """
    Authenticate user and create session.

    Authenticates user credentials and returns a session token.
    """
    result = auth_service.authenticate_user(
        db=db,
        email=request.email,
        password=request.password
    )

    if not result["success"]:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=result["error"]
        )

    # Create session token
    token = auth_service.create_session_token(result["user"]["id"])

    return {
        "success": True,
        "message": "Login successful",
        "user": result["user"],
        "profile": result["profile"],
        "token": token
    }


@router.post("/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Logout user and destroy session.

    In a stateless JWT system, this is typically handled on the client side.
    This endpoint can be used to notify the server of logout.
    """
    # In a real system, you might add the token to a blacklist
    return {
        "success": True,
        "message": "Logout successful"
    }


@router.get("/profile")
async def get_profile(current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """
    Get current user's profile.

    Retrieves the profile information for the authenticated user.
    """
    result = auth_service.get_user_profile(db=db, user_id=current_user_id)

    if not result["success"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=result["error"]
        )

    return {
        "success": True,
        "profile": result["profile"]
    }


@router.put("/profile")
async def update_profile(
    request: UpdateProfileRequest,
    current_user_id: str = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """
    Update current user's profile.

    Updates the profile information for the authenticated user.
    """
    result = auth_service.update_user_profile(
        db=db,
        user_id=current_user_id,
        software_experience=request.software_experience,
        hardware_experience=request.hardware_experience,
        background_preference=request.background_preference
    )

    if not result["success"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=result["error"]
        )

    return {
        "success": True,
        "profile": result["profile"]
    }