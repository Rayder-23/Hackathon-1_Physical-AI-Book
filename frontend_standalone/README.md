# Physical AI Book RAG Chatbot Frontend

This is the frontend for the Physical AI Book RAG Chatbot, designed to be embedded in static book sites with a seamless chat experience.

## Features

- Embedded chat interface alongside book content
- Two query modes: Full-book context and Selected-text only
- Text selection capture for focused queries
- Responsive design that works on all devices
- Real-time message streaming (simulated)
- Citation display for book references

## Components

### Files
- `index.html`: Main page with book content and chat interface
- `styles.css`: Styling for the book and chat components
- `chat-interface.js`: JavaScript for chat functionality and API integration

## Integration

To embed the chatbot in your static book site:

1. Include the CSS in your HTML:
```html
<link rel="stylesheet" href="styles.css">
```

2. Add the chat HTML structure to your page:
```html
<div class="chat-container">
    <div class="chat-header">
        <h3>Physical AI Book Assistant</h3>
        <div class="mode-selector">
            <button id="full-book-mode" class="mode-btn active">Full Book Context</button>
            <button id="selected-text-mode" class="mode-btn">Selected Text Only</button>
        </div>
    </div>

    <div id="chat-messages" class="chat-messages">
        <!-- Messages will be added here dynamically -->
    </div>

    <div class="chat-input-area">
        <div id="selected-text-preview" class="selected-text-preview" style="display: none;">
            <span class="preview-label">Selected text:</span>
            <span id="selected-text-content"></span>
            <button id="clear-selection" class="clear-btn">×</button>
        </div>
        <div class="input-container">
            <textarea id="user-input" placeholder="Ask a question about the Physical AI book..." rows="1"></textarea>
            <button id="send-btn" class="send-btn">Send</button>
        </div>
    </div>
</div>
```

3. Include the JavaScript:
```html
<script src="chat-interface.js"></script>
```

4. Update the API base URL in `chat-interface.js` to point to your backend:
```javascript
this.apiBaseUrl = 'https://your-backend-domain.com/api';
```

## Functionality

### Text Selection
The interface captures text selected by the user in the book content and allows asking questions specifically about that text when in "Selected Text Only" mode.

### Query Modes
- **Full Book Context**: Searches the entire book content for relevant information
- **Selected Text Only**: Restricts search to only the user-selected text

### Message Display
- User messages appear on the right
- Bot responses appear on the left with citations when available
- Typing indicators show when processing

## Customization

You can customize the appearance by modifying the CSS variables or classes. The interface is designed to match common book reading applications while maintaining visibility of the book content.

## API Integration

The frontend communicates with the backend API at `/api/chat` to process queries. The backend should implement the expected API endpoints as defined in the backend documentation.