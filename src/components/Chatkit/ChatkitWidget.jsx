import React, { useState, useEffect, useRef } from 'react';
import '../../css/chatkit-styles.css';

// Custom Chatkit widget implementation that works with our RAG backend
const ChatkitWidget = ({ roomId, userId }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('connecting');
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize the connection to our backend
  useEffect(() => {
    const initializeConnection = async () => {
      try {
        // Fetch the token from our backend - use environment variable or default
        const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
          ? process.env.REACT_APP_BACKEND_URL
          : 'http://localhost:8005';
        const res = await fetch(`${backendUrl}/api/chatkit/token`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ user_id: userId, session_id: roomId }),
          mode: 'cors',
        });

        if (!res.ok) {
          throw new Error(`Failed to get token: ${res.status} ${res.statusText}`);
        }

        const data = await res.json();
        const token = data.token;

        if (!token) {
          throw new Error('Invalid token response from server');
        }

        // Connection successful
        setConnectionStatus('connected');

        // Add a welcome message
        setMessages(prev => [...prev, {
          id: Date.now(),
          text: "Hello! I'm your Physical AI Book Assistant. Ask me anything about the Physical AI book!",
          sender: 'assistant',
          timestamp: new Date().toISOString()
        }]);
      } catch (error) {
        console.error('Error initializing chat:', error);
        setConnectionStatus('error');
      }
    };

    initializeConnection();
  }, [userId, roomId]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };


  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Send message to our RAG backend
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    // Add user message to UI immediately
    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      // Send to our backend API - use environment variable or default
      const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
        ? process.env.REACT_APP_BACKEND_URL
        : 'http://localhost:8005';
      const response = await fetch(`${backendUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: currentInput,
          session_id: roomId
        }),
      });

      if (!response.ok) {
        throw new Error(`Failed to get response: ${response.status}`);
      }

      const responseData = await response.json();

      // Add bot response to messages
      const botMessage = {
        id: Date.now() + 1,
        text: responseData.message || responseData.content || "I processed your request.",
        sender: 'assistant',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: 'assistant',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle pressing Enter to send message
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  if (!isOpen) {
    return (
      <button className="chatkit-launcher" onClick={toggleChat}>
        💬 Chat
      </button>
    );
  }


  if (connectionStatus === 'connecting') {
    return (
      <div className="chatkit-widget">
        <div className="chatkit-header">
          <h3>Physical AI Book Assistant</h3>
          <div className="chatkit-controls">
            <button className="close-btn" onClick={toggleChat}>✕</button>
          </div>
        </div>
        <div className="chatkit-content">
          <div className="chat-loading">Connecting to chat service...</div>
        </div>
      </div>
    );
  }

  if (connectionStatus === 'error') {
    return (
      <div className="chatkit-widget">
        <div className="chatkit-header">
          <h3>Physical AI Book Assistant</h3>
          <div className="chatkit-controls">
            <button className="close-btn" onClick={toggleChat}>✕</button>
          </div>
        </div>
        <div className="chatkit-content">
          <div className="chat-error">Connection error. Please try again.</div>
        </div>
      </div>
    );
  }

  return (
    <div className="chatkit-widget">
      <div className="chatkit-header">
        <h3>Physical AI Book Assistant</h3>
        <div className="chatkit-controls">
          <button className="close-btn" onClick={toggleChat}>✕</button>
        </div>
      </div>
      <div className="chatkit-content">
        <div className="messages-container">
          {messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender === 'user' ? 'user-message' : 'assistant-message'}`}
            >
              <div className="message-text">{message.text}</div>
              <div className="message-timestamp">
                {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className="message assistant-message">
              <div className="message-text">⏳ Thinking...</div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>
        <div className="input-container">
          <textarea
            className="message-input"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask about the Physical AI book..."
            rows="1"
          />
          <button
            className="send-button"
            onClick={sendMessage}
            disabled={isLoading || !inputValue.trim()}
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatkitWidget;