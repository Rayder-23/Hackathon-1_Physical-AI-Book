from typing import Dict, Any, Optional
import os
from datetime import datetime
from sqlalchemy.orm import Session
from models.models import UserSession, Conversation, ChatMessage, RagQuery
from services.session_service import session_service
from agents.agent import rag_agent, QueryMode
from data.vector_store import vector_store
from data.embeddings import embeddings_service


class ChatkitAdapter:
    """
    Adapter class to interface between Chatkit and the RAG system.
    Handles incoming messages from Chatkit, processes them through the RAG system,
    and manages session and conversation state.
    """

    def __init__(self, db: Session):
        self.db = db

    def handle_incoming_message(self, chatkit_session_id: str, room_id: str, sender_id: str,
                               message_content: str, sender_type: str = 'user') -> Dict[str, Any]:
        """
        Handle an incoming message from Chatkit.

        Args:
            chatkit_session_id: Chatkit session identifier
            room_id: Chatkit room identifier
            sender_id: User or system identifier
            message_content: The message content
            sender_type: Type of sender ('user', 'bot', 'system')

        Returns:
            Dict containing the response and metadata
        """
        # Get or create session
        session = session_service.get_session_by_chatkit_id(self.db, chatkit_session_id)
        if not session:
            session = session_service.create_session(
                db=self.db,
                chatkit_session_id=chatkit_session_id
            )

        # Get or create conversation
        conversation = self._get_or_create_conversation(room_id, session.id)

        # Create chat message record
        chat_message = self._create_chat_message(
            room_id=room_id,
            sender_id=sender_id,
            sender_type=sender_type,
            content=message_content,
            conversation_id=conversation.id
        )

        # Process the message through RAG if it's from a user
        response = None
        if sender_type == 'user':
            # Get conversation history for context in follow-up questions
            conversation_history = self.get_conversation_history(room_id)

            # Use the existing rag_agent to process the query with conversation history
            agent_response = rag_agent.process_query(
                user_query=message_content,
                mode=QueryMode.FULL_BOOK,  # Default to full-book mode
                conversation_history=conversation_history  # Pass conversation history for context
            )

            # Extract information from agent response
            response_content = agent_response.content
            response_citations = agent_response.citations
            response_tokens = agent_response.token_usage

            # Create RAG query record
            rag_query = self._create_rag_query(
                session_id=session.id,
                query_text=message_content,
                response_text=response_content,
                context_documents=response_citations,
                response_time_ms=0,  # Not tracked in agent
                relevance_score=0.0  # Not directly available from agent
            )

            # Create response message
            if agent_response:
                response_message = self._create_chat_message(
                    room_id=room_id,
                    sender_id='chatbot',
                    sender_type='bot',
                    content=response_content,
                    conversation_id=conversation.id,
                    related_query_id=rag_query.id  # Link to the RagQuery
                )

        # Update conversation metadata
        self._update_conversation_metadata(conversation.id)

        # Update session activity
        session_service.update_session_activity(self.db, session.id)

        return {
            'session_id': session.id,
            'conversation_id': conversation.id,
            'response': response_content if response_content else response,
            'message_id': chat_message.id
        }

    def _get_or_create_conversation(self, room_id: str, session_id: str) -> Conversation:
        """
        Get an existing conversation or create a new one if it doesn't exist.

        Args:
            room_id: Chatkit room identifier
            session_id: Session UUID

        Returns:
            Conversation object
        """
        # Try to find existing conversation
        conversation = self.db.query(Conversation).filter(
            Conversation.chatkit_room_id == room_id
        ).first()

        if not conversation:
            # Create new conversation
            conversation = Conversation(
                chatkit_room_id=room_id,
                session_id=session_id,
                title="New Conversation"  # Will be updated with first query later if needed
            )
            self.db.add(conversation)
            self.db.commit()
            self.db.refresh(conversation)

        return conversation

    def _create_chat_message(self, room_id: str, sender_id: str, sender_type: str,
                             content: str, conversation_id: str,
                             related_query_id: Optional[str] = None) -> ChatMessage:
        """
        Create a chat message record in the database.

        Args:
            room_id: Chatkit room identifier
            sender_id: Sender identifier
            sender_type: Type of sender
            content: Message content
            conversation_id: Conversation UUID
            related_query_id: Optional related query ID

        Returns:
            ChatMessage object
        """
        chat_message = ChatMessage(
            room_id=room_id,
            sender_id=sender_id,
            sender_type=sender_type,
            content=content,
            message_type='text',  # Default to text
            related_query_id=related_query_id,
            conversation_id=conversation_id
        )

        self.db.add(chat_message)
        self.db.commit()
        self.db.refresh(chat_message)

        return chat_message

    def _create_rag_query(self, session_id: str, query_text: str, response_text: str,
                          context_documents: list, response_time_ms: int,
                          relevance_score: float) -> RagQuery:
        """
        Create a RAG query record in the database.

        Args:
            session_id: Session UUID
            query_text: Original query text
            response_text: Generated response
            context_documents: List of context documents used
            response_time_ms: Time taken to process the query
            relevance_score: Relevance score of the response

        Returns:
            RagQuery object
        """
        rag_query = RagQuery(
            session_id=session_id,
            query_text=query_text,
            response_text=response_text,
            context_documents=context_documents if context_documents else [],
            response_time_ms=response_time_ms,
            relevance_score=relevance_score
        )

        self.db.add(rag_query)
        self.db.commit()
        self.db.refresh(rag_query)

        return rag_query

    def _update_conversation_metadata(self, conversation_id: str):
        """
        Update conversation metadata like last message time and message count.

        Args:
            conversation_id: Conversation UUID
        """
        conversation = self.db.query(Conversation).filter(
            Conversation.id == conversation_id
        ).first()

        if conversation:
            # Update last message time
            conversation.last_message_at = datetime.utcnow()

            # Update message count
            message_count = self.db.query(ChatMessage).filter(
                ChatMessage.conversation_id == conversation_id
            ).count()
            conversation.message_count = message_count

            self.db.commit()

    def get_conversation_history(self, room_id: str) -> list:
        """
        Retrieve the conversation history for a given room.

        Args:
            room_id: Chatkit room identifier

        Returns:
            List of messages in the conversation
        """
        conversation = self.db.query(Conversation).filter(
            Conversation.chatkit_room_id == room_id
        ).first()

        if not conversation:
            return []

        messages = self.db.query(ChatMessage).filter(
            ChatMessage.conversation_id == conversation.id
        ).order_by(ChatMessage.timestamp).all()

        return [
            {
                'id': str(msg.id),
                'roomId': msg.room_id,
                'senderId': msg.sender_id,
                'senderType': msg.sender_type,
                'content': msg.content,
                'timestamp': msg.timestamp.isoformat() if msg.timestamp else None,
                'messageType': msg.message_type
            }
            for msg in messages
        ]

    def create_token(self, user_id: str, session_id: str = None):
        """
        Create an authentication token for Chatkit.

        Args:
            user_id: User identifier
            session_id: Optional session identifier

        Returns:
            Token for Chatkit authentication
        """
        # This would typically involve calling the Chatkit SDK to generate a token
        # For now, we'll return a placeholder - the actual implementation would
        # use the openai-chatkit Python SDK
        import jwt
        import time

        # In a real implementation, we would use the Chatkit SDK:
        # from chatkit import Chatkit
        # token = chatkit.authenticate({
        #     "user_id": user_id,
        #     "session_id": session_id
        # })

        # For now, return a simple JWT token as a placeholder
        payload = {
            "user_id": user_id,
            "session_id": session_id,
            "exp": int(time.time()) + 86400  # 24 hours
        }

        # In a real implementation, you'd use the actual Chatkit secret key
        secret = os.getenv("CHATKIT_SECRET_KEY", "default_secret_for_demo")
        token = jwt.encode(payload, secret, algorithm="HS256")

        return {"token": token}