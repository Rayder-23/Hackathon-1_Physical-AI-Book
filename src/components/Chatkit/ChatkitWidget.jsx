import React, { useState, useCallback, useRef } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import '../../css/chatkit-styles.css';

// Chatkit widget implementation using the actual @openai/chatkit-react library

const ChatkitWidget = ({ roomId, userId }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('connecting');
  const chatkitRef = useRef(null);

  const { control, isInitialized, error: chatkitError } = useChatKit({
    api: {
      async getClientSecret(existing) {
        try {
          if (existing) {
            // implement session refresh
          }

          // Create a timeout promise
          const timeoutPromise = new Promise((_, reject) => {
            setTimeout(() => reject(new Error('Request timeout')), 10000); // 10 second timeout
          });

          // Race the fetch request against the timeout
          const res = await Promise.race([
            fetch('http://localhost:8005/api/chatkit/token', {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify({ user_id: userId, session_id: roomId }),
              mode: 'cors',
            }),
            timeoutPromise
          ]);

          if (!res.ok) {
            throw new Error(`Failed to get token: ${res.status} ${res.statusText}`);
          }

          const data = await res.json();
          // The backend returns a token field, but Chatkit might expect client_secret
          // Check multiple possible response formats
          const token = data.token || data.client_secret || data.access_token || data.data?.token;

          if (!token) {
            console.error('Invalid token response from server:', data);
            throw new Error('Invalid token response from server. Expected "token", "client_secret", or "access_token" field.');
          }

          return token;
        } catch (error) {
          console.error('Error getting client secret:', error);
          setConnectionStatus('error');
          throw error;
        }
      },
    },
  });

  // Set connection status based on initialization
  React.useEffect(() => {
    if (isInitialized) {
      setConnectionStatus('connected');
    } else if (chatkitError) {
      setConnectionStatus('error');
    }
  }, [isInitialized, chatkitError]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const toggleMinimize = () => {
    setIsMinimized(!isMinimized);
  };

  // Callbacks for chat events
  const handleThreadChange = useCallback((threadId) => {
    console.log('Thread changed:', threadId);
  }, []);

  const handleResponseCompleted = useCallback(() => {
    console.log('Response completed');
  }, []);

  const handleProfileUpdate = useCallback((profile) => {
    console.log('Profile updated:', profile);
  }, []);

  // Handle custom actions
  const handleWidgetActionComplete = useCallback(() => {
    console.log('Widget action completed');
  }, []);

  // Set up chatkit reference when ready
  const handleChatKitReady = useCallback((chatkit) => {
    chatkitRef.current = chatkit;
  }, []);

  if (!isOpen) {
    return (
      <button className="chatkit-launcher" onClick={toggleChat}>
        💬 Chat
      </button>
    );
  }

  if (isMinimized) {
    return (
      <div className="chatkit-widget minimized">
        <div className="chatkit-header minimized" onClick={toggleMinimize}>
          <span>💬 Chat</span>
          <button className="minimize-btn">+</button>
        </div>
      </div>
    );
  }

  // Show loading while connecting
  if (connectionStatus === 'connecting') {
    return (
      <div className="chatkit-widget">
        <div className="chatkit-header">
          <h3>Physical AI Book Assistant</h3>
          <div className="chatkit-controls">
            <button className="minimize-btn" onClick={toggleMinimize}>−</button>
            <button className="close-btn" onClick={toggleChat}>✕</button>
          </div>
        </div>
        <div className="chatkit-content">
          <div className="chat-loading">Connecting to chat service...</div>
        </div>
      </div>
    );
  }

  // Show error state
  if (connectionStatus === 'error') {
    return (
      <div className="chatkit-widget">
        <div className="chatkit-header">
          <h3>Physical AI Book Assistant</h3>
          <div className="chatkit-controls">
            <button className="minimize-btn" onClick={toggleMinimize}>−</button>
            <button className="close-btn" onClick={toggleChat}>✕</button>
          </div>
        </div>
        <div className="chatkit-content">
          <div className="chat-error">Connection error. Please try again.</div>
        </div>
      </div>
    );
  }

  // Render the ChatKit component with the control from useChatKit
  return (
    <div className="chatkit-widget">
      <div className="chatkit-header">
        <h3>Physical AI Book Assistant</h3>
        <div className="chatkit-controls">
          <button className="minimize-btn" onClick={toggleMinimize}>−</button>
          <button className="close-btn" onClick={toggleChat}>✕</button>
        </div>
      </div>
      <div className="chatkit-content">
        {control ? (
          <ChatKit
            control={control}
            className="chatkit-container"
            style={{ height: '400px', width: '100%' }}
          />
        ) : (
          <div className="chat-loading">Initializing chat interface...</div>
        )}
      </div>
    </div>
  );
};

export default ChatkitWidget;