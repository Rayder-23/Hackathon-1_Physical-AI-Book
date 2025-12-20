import React, { useState, useEffect } from 'react';
import ChatkitProvider from './ChatkitProvider';
import ChatkitWidget from './ChatkitWidget';
import '../../css/chatkit-styles.css';
import config from '../../utils/config';

const ChatInterface = () => {
  const [roomId, setRoomId] = useState(null);
  const [userId, setUserId] = useState(null);
  const [isInitialized, setIsInitialized] = useState(false);

  // Initialize chat on component mount
  useEffect(() => {
    // In a real implementation, this would create or get a session/room
    // For now, we'll use a default room ID in UUID format to match backend expectations
    let storedRoomId = localStorage.getItem('chatkit_room_id');

    // If no room ID exists in localStorage, create a new UUID
    if (!storedRoomId) {
      // Generate a proper UUID for the room
      storedRoomId = 'room_' + crypto.randomUUID?.() || 'room_' + ([1e7]+-1e3+-4e3+-8e3+-1e11).replace(/[018]/g, c =>
        (c ^ crypto.getRandomValues(new Uint8Array(1))[0] & 15 >> c / 4).toString(16)
      );
      localStorage.setItem('chatkit_room_id', storedRoomId);
    }
    setRoomId(storedRoomId);

    // Get or create user ID (could be from auth context or generated)
    let currentUserId = localStorage.getItem('chatkit_user_id');
    if (!currentUserId) {
      currentUserId = 'user_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
      localStorage.setItem('chatkit_user_id', currentUserId);
    }
    setUserId(currentUserId);

    setIsInitialized(true);
  }, []);

  // Define token provider function
  const getToken = async (userId) => {
    try {
      // Use the configured backend URL
      const response = await fetch(`${config.BACKEND_URL}/api/chatkit/token`, {
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