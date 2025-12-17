import React, { useState, useEffect, useRef } from 'react';
import './ChatInterface.css';

const ChatInterface = ({ bookContentRef }) => {
  const [messages, setMessages] = useState([
    { id: 1, type: 'bot', content: 'Hello! I\'m your Physical AI book assistant. You can ask me questions about the book content, and I\'ll provide answers based on the book\'s text. Select text on the page to ask questions specifically about that content.', timestamp: new Date(), citations: [] }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentMode, setCurrentMode] = useState('full-book');
  const [selectedText, setSelectedText] = useState(null);
  const [sessionId, setSessionId] = useState(null);
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Setup text selection listener
  useEffect(() => {
    if (bookContentRef?.current) {
      const handleTextSelection = () => {
        const selectedText = window.getSelection().toString().trim();
        if (selectedText && selectedText.length > 10) { // Only consider meaningful selections
          setSelectedText(selectedText);
          if (currentMode === 'selected-text') {
            addBotMessage(`I'll answer your questions based only on the selected text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`);
          }
        }
      };

      const contentElement = bookContentRef.current;
      contentElement.addEventListener('mouseup', handleTextSelection);

      return () => {
        contentElement.removeEventListener('mouseup', handleTextSelection);
      };
    }
  }, [bookContentRef, currentMode]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const addBotMessage = (content, citations = []) => {
    const newMessage = {
      id: Date.now(),
      type: 'bot',
      content,
      timestamp: new Date(),
      citations
    };
    setMessages(prev => [...prev, newMessage]);
  };

  const addUserMessage = (content) => {
    const newMessage = {
      id: Date.now(),
      type: 'user',
      content,
      timestamp: new Date(),
      citations: []
    };
    setMessages(prev => [...prev, newMessage]);
  };

  const autoResizeTextarea = () => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = Math.min(textareaRef.current.scrollHeight, 100) + 'px';
    }
  };

  const handleSendMessage = async () => {
    const message = inputValue.trim();
    if (!message) return;

    // Add user message to UI
    addUserMessage(message);
    setInputValue('');
    autoResizeTextarea();

    // Disable input while processing
    setIsLoading(true);

    try {
      // Show typing indicator
      const typingMessage = {
        id: 'typing',
        type: 'typing',
        content: '',
        timestamp: new Date(),
        citations: []
      };
      setMessages(prev => [...prev, typingMessage]);

      // Prepare the request payload
      const requestBody = {
        message: message,
        mode: currentMode,
        selected_text: currentMode === 'selected-text' ? selectedText : null
      };

      // Include session ID if available
      if (sessionId) {
        requestBody.session_id = sessionId;
      }

      // Call the backend API
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      // Remove typing indicator
      setMessages(prev => prev.filter(msg => msg.id !== 'typing'));

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Update session ID if new one was returned
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add bot response to UI
      addBotMessage(data.message, data.citations || []);
    } catch (error) {
      console.error('Error sending message:', error);

      // Remove typing indicator
      setMessages(prev => prev.filter(msg => msg.id !== 'typing'));

      // Show error message
      addBotMessage('Sorry, I encountered an error processing your request. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const setMode = (mode) => {
    setCurrentMode(mode);

    // Show help message when switching mode
    if (mode === 'selected-text' && selectedText) {
      addBotMessage(`Selected text mode activated. I'll answer based only on: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`);
    } else if (mode === 'full-book') {
      addBotMessage('Full book context mode activated. I can use all book content to answer your questions.');
    }
  };

  const clearSelection = () => {
    setSelectedText(null);
    if (currentMode === 'selected-text') {
      setMode('full-book');
    }
    window.getSelection().removeAllRanges(); // Clear any existing selection
  };

  return (
    <div className="chat-container">
      <div className="chat-header">
        <h3>Physical AI Book Assistant</h3>
        <div className="mode-selector">
          <button
            className={`mode-btn ${currentMode === 'full-book' ? 'active' : ''}`}
            onClick={() => setMode('full-book')}
          >
            Full Book Context
          </button>
          <button
            className={`mode-btn ${currentMode === 'selected-text' ? 'active' : ''}`}
            onClick={() => setMode('selected-text')}
          >
            Selected Text Only
          </button>
        </div>
      </div>

      <div className="chat-messages">
        {messages.filter(msg => msg.id !== 'typing').map((message) => (
          <div key={message.id} className={`message ${message.type}-message`}>
            <div className="message-content">
              {message.content}
            </div>
            {message.citations && message.citations.length > 0 && (
              <div className="citations">
                {message.citations.slice(0, 3).map((citation, idx) => (
                  <div key={idx} className="citation">
                    {citation.chapter && `Chapter: ${citation.chapter} `}
                    {citation.section && `Section: ${citation.section} `}
                    {citation.page && `Page: ${citation.page}`}
                  </div>
                ))}
              </div>
            )}
            <div className="message-meta">Book Assistant • Just now</div>
          </div>
        ))}

        {messages.some(msg => msg.id === 'typing') && (
          <div className="bot-typing">
            <div className="loading"></div>
            <span>Thinking...</span>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <span className="preview-label">Selected text:</span>
          <span id="selected-text-content">"{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"</span>
          <button className="clear-btn" onClick={clearSelection}>×</button>
        </div>
      )}

      <div className="chat-input-area">
        <div className="input-container">
          <textarea
            ref={textareaRef}
            value={inputValue}
            onChange={(e) => {
              setInputValue(e.target.value);
              autoResizeTextarea();
            }}
            onKeyDown={handleKeyDown}
            placeholder="Ask a question about the Physical AI book..."
            rows="1"
            disabled={isLoading}
          />
          <button
            className="send-btn"
            onClick={handleSendMessage}
            disabled={isLoading || !inputValue.trim()}
          >
            Send
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatInterface;