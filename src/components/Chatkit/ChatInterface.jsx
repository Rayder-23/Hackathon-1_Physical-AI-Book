import React, { useState, useEffect } from 'react';
import ChatkitProvider from './ChatkitProvider';
import ChatkitWidget from './ChatkitWidget';
import '../../css/chatkit-styles.css';

const ChatInterface = () => {
  const [roomId, setRoomId] = useState(null);
  const [userId, setUserId] = useState(null);
  const [isInitialized, setIsInitialized] = useState(false);

  // Initialize chat on component mount
  useEffect(() => {
    // In a real implementation, this would create or get a session/room
    // For now, we'll use a default room ID
    // In Docusaurus, we might want to create a persistent room for the user
    const storedRoomId = localStorage.getItem('chatkit_room_id');
    const defaultRoomId = storedRoomId || 'docusaurus-chat-room';
    setRoomId(defaultRoomId);

    // Get or create user ID (could be from auth context or generated)
    let currentUserId = localStorage.getItem('chatkit_user_id');
    if (!currentUserId) {
      currentUserId = 'user_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
      localStorage.setItem('chatkit_user_id', currentUserId);
    }
    setUserId(currentUserId);

    // Store room ID in localStorage to persist across page navigation
    if (!storedRoomId) {
      localStorage.setItem('chatkit_room_id', defaultRoomId);
    }

    setIsInitialized(true);
  }, []);

  // Define token provider function
  const getToken = async (userId) => {
    try {
      const response = await fetch('/api/chatkit/token', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ user_id: userId, session_id: roomId }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data.token;
    } catch (error) {
      console.error('Error getting token:', error);
      throw error;
    }
  };

  if (!isInitialized) {
    return (
      <div className="chat-interface">
        <div className="chat-loading">
          Initializing chat service...
        </div>
      </div>
    );
  }

  return (
    <div className="chat-interface">
      {roomId && userId && (
        <ChatkitWidget roomId={roomId} userId={userId} />
      )}
    </div>
  );
};

export default ChatInterface;