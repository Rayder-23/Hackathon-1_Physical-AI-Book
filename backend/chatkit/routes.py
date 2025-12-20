from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session
from typing import Dict, Any
import os
import jwt
from datetime import datetime, timedelta

from database.database import get_db
from chatkit.adapter import ChatkitAdapter
from services.session_service import session_service

router = APIRouter(prefix="/chatkit", tags=["chatkit"])

@router.post("/token")
async def generate_chatkit_token(
    request: Request,
    db: Session = Depends(get_db)
):
    """
    Generate an authentication token for Chatkit.

    Request body should contain:
    - user_id: The user identifier
    - session_id: Optional session identifier
    """
    try:
        body = await request.json()
        user_id = body.get("user_id")
        session_id = body.get("session_id")

        if not user_id:
            raise HTTPException(status_code=400, detail="user_id is required")

        # Create or get session
        session = session_service.get_session_by_chatkit_id(db, session_id) if session_id else None

        if not session:
            session = session_service.create_session(
                db=db,
                user_id=user_id,
                chatkit_session_id=session_id
            )

        # In a real implementation, you would use the Chatkit SDK to generate a token
        # For now, we'll return a placeholder JWT token
        payload = {
            "user_id": user_id,
            "session_id": str(session.id) if session else session_id,
            "exp": datetime.utcnow() + timedelta(hours=24)  # 24 hour expiration
        }

        secret = os.getenv("CHATKIT_SECRET_KEY", "default_secret_for_demo")
        token = jwt.encode(payload, secret, algorithm="HS256")

        return {
            "token": token,
            "session_id": str(session.id) if session else session_id
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating token: {str(e)}")


@router.post("/message")
async def handle_chatkit_message(
    request: Request,
    db: Session = Depends(get_db)
):
    """
    Handle incoming messages from Chatkit.

    Request body should contain:
    - chatkit_session_id: Chatkit session identifier
    - room_id: Chatkit room identifier
    - sender_id: Sender identifier
    - message_content: The message content
    - sender_type: Type of sender ('user', 'bot', 'system')
    """
    try:
        body = await request.json()

        chatkit_session_id = body.get("chatkit_session_id")
        room_id = body.get("room_id")
        sender_id = body.get("sender_id")
        message_content = body.get("message_content")
        sender_type = body.get("sender_type", "user")

        if not all([chatkit_session_id, room_id, sender_id, message_content]):
            raise HTTPException(
                status_code=400,
                detail="chatkit_session_id, room_id, sender_id, and message_content are required"
            )

        # Use the Chatkit adapter to handle the message
        adapter = ChatkitAdapter(db)
        result = adapter.handle_incoming_message(
            chatkit_session_id=chatkit_session_id,
            room_id=room_id,
            sender_id=sender_id,
            message_content=message_content,
            sender_type=sender_type
        )

        return result

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error handling message: {str(e)}")


@router.get("/history/{room_id}")
async def get_conversation_history(
    room_id: str,
    db: Session = Depends(get_db)
):
    """
    Get conversation history for a specific room.

    Args:
        room_id: Chatkit room identifier
    """
    try:
        adapter = ChatkitAdapter(db)
        history = adapter.get_conversation_history(room_id)

        return {
            "room_id": room_id,
            "messages": history
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving history: {str(e)}")


@router.post("/create-session")
async def create_chatkit_session(
    request: Request,
    db: Session = Depends(get_db)
):
    """
    Create a new chat session.

    Request body can contain:
    - user_id: Optional user identifier for authenticated users
    """
    try:
        body = await request.json()
        user_id = body.get("user_id")  # Optional for guest sessions

        session = session_service.create_session(
            db=db,
            user_id=user_id
        )

        return {
            "session_id": str(session.id),
            "chatkit_session_id": session.chatkit_session_id,
            "user_id": session.user_id,
            "expires_at": session.expires_at.isoformat() if session.expires_at else None
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating session: {str(e)}")