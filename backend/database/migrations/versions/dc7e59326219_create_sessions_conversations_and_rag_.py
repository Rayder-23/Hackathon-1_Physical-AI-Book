"""Create sessions, conversations, and rag_queries tables for Chatkit integration

Revision ID: dc7e59326219
Revises:
Create Date: 2025-12-19 23:35:00.701016

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql
from datetime import datetime
import uuid

# revision identifiers, used by Alembic.
revision = 'dc7e59326219'
down_revision = None
branch_labels = None
depends_on = None


def upgrade():
    # Create sessions table
    op.create_table('sessions',
        sa.Column('id', postgresql.UUID(as_uuid=True), server_default=sa.text('gen_random_uuid()'), nullable=False),
        sa.Column('user_id', sa.String(255), nullable=True),
        sa.Column('chatkit_session_id', sa.String(255), nullable=False),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('NOW()'), nullable=True),
        sa.Column('last_activity_at', sa.DateTime(), server_default=sa.text('NOW()'), nullable=True),
        sa.Column('expires_at', sa.DateTime(), nullable=True),
        sa.Column('is_active', sa.Boolean(), server_default=sa.text('true'), nullable=True),
        sa.PrimaryKeyConstraint('id', name='sessions_pkey')
    )

    # Create indexes for sessions table
    op.create_index('idx_sessions_user_id', 'sessions', ['user_id'], unique=False)
    op.create_index('idx_sessions_chatkit_session_id', 'sessions', ['chatkit_session_id'], unique=False)
    op.create_index('idx_sessions_expires_at', 'sessions', ['expires_at'], unique=False)

    # Create conversations table
    op.create_table('conversations',
        sa.Column('id', postgresql.UUID(as_uuid=True), server_default=sa.text('gen_random_uuid()'), nullable=False),
        sa.Column('chatkit_room_id', sa.String(255), nullable=False),
        sa.Column('session_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('title', sa.String(500), nullable=True),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('NOW()'), nullable=True),
        sa.Column('last_message_at', sa.DateTime(), server_default=sa.text('NOW()'), nullable=True),
        sa.Column('message_count', sa.Integer(), server_default=sa.text('0'), nullable=True),
        sa.Column('is_active', sa.Boolean(), server_default=sa.text('true'), nullable=True),
        sa.ForeignKeyConstraint(['session_id'], ['sessions.id'], name='conversations_session_id_fkey'),
        sa.PrimaryKeyConstraint('id', name='conversations_pkey')
    )

    # Create indexes for conversations table
    op.create_index('idx_conversations_session_id', 'conversations', ['session_id'], unique=False)
    op.create_index('idx_conversations_chatkit_room_id', 'conversations', ['chatkit_room_id'], unique=False)

    # Create chat_messages table
    op.create_table('chat_messages',
        sa.Column('id', postgresql.UUID(as_uuid=True), server_default=sa.text('gen_random_uuid()'), nullable=False),
        sa.Column('room_id', sa.String(255), nullable=False),
        sa.Column('sender_id', sa.String(255), nullable=False),
        sa.Column('sender_type', sa.String(20), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('timestamp', sa.DateTime(), server_default=sa.text('NOW()'), nullable=True),
        sa.Column('message_type', sa.String(20), nullable=False),
        sa.Column('related_query_id', postgresql.UUID(as_uuid=True), nullable=True),
        sa.Column('conversation_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.ForeignKeyConstraint(['conversation_id'], ['conversations.id'], name='chat_messages_conversation_id_fkey'),
        sa.PrimaryKeyConstraint('id', name='chat_messages_pkey')
    )

    # Create rag_queries table
    op.create_table('rag_queries',
        sa.Column('id', postgresql.UUID(as_uuid=True), server_default=sa.text('gen_random_uuid()'), nullable=False),
        sa.Column('session_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('query_text', sa.Text(), nullable=False),
        sa.Column('query_vector', sa.String(), nullable=True),
        sa.Column('context_documents', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('response_text', sa.Text(), nullable=True),
        sa.Column('timestamp', sa.DateTime(), server_default=sa.text('NOW()'), nullable=True),
        sa.Column('response_time_ms', sa.Integer(), nullable=True),
        sa.Column('relevance_score', sa.Float(), nullable=True),
        sa.ForeignKeyConstraint(['session_id'], ['sessions.id'], name='rag_queries_session_id_fkey'),
        sa.PrimaryKeyConstraint('id', name='rag_queries_pkey')
    )

    # Create indexes for rag_queries table
    op.create_index('idx_rag_queries_session_id', 'rag_queries', ['session_id'], unique=False)
    op.create_index('idx_rag_queries_timestamp', 'rag_queries', ['timestamp'], unique=False)


def downgrade():
    # Drop indexes first
    op.drop_index('idx_rag_queries_timestamp', table_name='rag_queries')
    op.drop_index('idx_rag_queries_session_id', table_name='rag_queries')
    op.drop_index('idx_conversations_chatkit_room_id', table_name='conversations')
    op.drop_index('idx_conversations_session_id', table_name='conversations')
    op.drop_index('idx_sessions_expires_at', table_name='sessions')
    op.drop_index('idx_sessions_chatkit_session_id', table_name='sessions')
    op.drop_index('idx_sessions_user_id', table_name='sessions')

    # Drop tables
    op.drop_table('rag_queries')
    op.drop_table('chat_messages')
    op.drop_table('conversations')
    op.drop_table('sessions')