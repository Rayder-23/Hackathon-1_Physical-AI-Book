// Chat interface for Physical AI Book RAG Chatbot
class BookChatInterface {
    constructor() {
        this.currentMode = 'full-book'; // Default mode
        this.currentSessionId = null;
        this.selectedText = null;

        // DOM elements
        this.chatMessages = document.getElementById('chat-messages');
        this.userInput = document.getElementById('user-input');
        this.sendBtn = document.getElementById('send-btn');
        this.selectedTextPreview = document.getElementById('selected-text-preview');
        this.selectedTextContent = document.getElementById('selected-text-content');
        this.clearSelectionBtn = document.getElementById('clear-selection');
        this.fullBookModeBtn = document.getElementById('full-book-mode');
        this.selectedTextModeBtn = document.getElementById('selected-text-mode');

        // API configuration
        this.apiBaseUrl = 'http://localhost:8000/api'; // Update this to your backend URL

        this.initializeEventListeners();
        this.setupTextSelection();
    }

    initializeEventListeners() {
        // Send button click
        this.sendBtn.addEventListener('click', () => this.sendMessage());

        // Enter key in textarea (with Shift+Enter for new line)
        this.userInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.sendMessage();
            }
        });

        // Mode selection
        this.fullBookModeBtn.addEventListener('click', () => this.setMode('full-book'));
        this.selectedTextModeBtn.addEventListener('click', () => this.setMode('selected-text'));

        // Clear selection
        this.clearSelectionBtn.addEventListener('click', () => this.clearSelection());

        // Auto-resize textarea
        this.userInput.addEventListener('input', this.autoResizeTextarea.bind(this));
    }

    setupTextSelection() {
        // Set up text selection across the book content
        const bookContent = document.querySelector('main');
        if (bookContent) {
            bookContent.addEventListener('mouseup', this.handleTextSelection.bind(this));
        }
    }

    handleTextSelection() {
        const selectedText = window.getSelection().toString().trim();

        if (selectedText) {
            this.selectedText = selectedText;
            this.updateSelectedTextPreview(selectedText);

            // If in selected-text mode, show a confirmation message
            if (this.currentMode === 'selected-text') {
                this.addBotMessage(`I'll answer your questions based only on the selected text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`);
            }
        }
    }

    setMode(mode) {
        this.currentMode = mode;

        // Update UI
        this.fullBookModeBtn.classList.toggle('active', mode === 'full-book');
        this.selectedTextModeBtn.classList.toggle('active', mode === 'selected-text');

        // Show help message when switching to selected-text mode
        if (mode === 'selected-text' && this.selectedText) {
            this.addBotMessage(`Selected text mode activated. I'll answer based only on: "${this.selectedText.substring(0, 100)}${this.selectedText.length > 100 ? '...' : ''}"`);
        } else if (mode === 'full-book') {
            this.addBotMessage('Full book context mode activated. I can use all book content to answer your questions.');
        }
    }

    updateSelectedTextPreview(text) {
        if (text) {
            this.selectedTextContent.textContent = `"${text.substring(0, 80)}${text.length > 80 ? '...' : ''}"`;
            this.selectedTextPreview.style.display = 'flex';
        } else {
            this.selectedTextPreview.style.display = 'none';
        }
    }

    clearSelection() {
        this.selectedText = null;
        this.updateSelectedTextPreview('');
        window.getSelection().removeAllRanges(); // Clear any existing selection

        if (this.currentMode === 'selected-text') {
            this.setMode('full-book'); // Switch back to full-book mode
        }
    }

    autoResizeTextarea() {
        this.userInput.style.height = 'auto';
        this.userInput.style.height = Math.min(this.userInput.scrollHeight, 100) + 'px';
    }

    async sendMessage() {
        const message = this.userInput.value.trim();
        if (!message) return;

        // Disable input while processing
        this.userInput.disabled = true;
        this.sendBtn.disabled = true;

        // Add user message to UI
        this.addUserMessage(message);

        // Clear input and reset height
        this.userInput.value = '';
        this.autoResizeTextarea();

        try {
            // Show typing indicator
            const typingIndicator = this.addBotTypingIndicator();

            // Prepare the request payload
            const requestBody = {
                message: message,
                mode: this.currentMode,
                selected_text: this.currentMode === 'selected-text' ? this.selectedText : null
            };

            // Include session ID if available
            if (this.currentSessionId) {
                requestBody.session_id = this.currentSessionId;
            }

            // Call the backend API
            const response = await fetch(`${this.apiBaseUrl}/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(requestBody)
            });

            if (!response.ok) {
                throw new Error(`API request failed with status ${response.status}`);
            }

            const data = await response.json();

            // Update session ID if new one was returned
            if (data.session_id && !this.currentSessionId) {
                this.currentSessionId = data.session_id;
            }

            // Remove typing indicator
            this.removeTypingIndicator(typingIndicator);

            // Add bot response to UI
            this.addBotMessage(data.message, data.citations);

        } catch (error) {
            console.error('Error sending message:', error);

            // Remove typing indicator
            this.removeTypingIndicator();

            // Show error message
            this.addBotMessage('Sorry, I encountered an error processing your request. Please try again.');
        } finally {
            // Re-enable input
            this.userInput.disabled = false;
            this.sendBtn.disabled = false;
            this.userInput.focus();
        }
    }

    addUserMessage(message) {
        const messageDiv = document.createElement('div');
        messageDiv.className = 'message user-message';

        messageDiv.innerHTML = `
            <div class="message-content">${this.escapeHtml(message)}</div>
            <div class="message-meta">You • Just now</div>
        `;

        this.chatMessages.appendChild(messageDiv);
        this.scrollToBottom();
    }

    addBotMessage(message, citations = null) {
        const messageDiv = document.createElement('div');
        messageDiv.className = 'message bot-message';

        let citationsHtml = '';
        if (citations && citations.length > 0) {
            citationsHtml = '<div class="citations">';
            citations.slice(0, 3).forEach(citation => { // Show up to 3 citations
                citationsHtml += `<div class="citation">`;
                if (citation.chapter) citationsHtml += `Chapter: ${citation.chapter} `;
                if (citation.section) citationsHtml += `Section: ${citation.section} `;
                if (citation.page) citationsHtml += `Page: ${citation.page}`;
                citationsHtml += `</div>`;
            });
            citationsHtml += '</div>';
        }

        messageDiv.innerHTML = `
            <div class="message-content">${this.formatMessageContent(message)}</div>
            ${citationsHtml}
            <div class="message-meta">Book Assistant • Just now</div>
        `;

        this.chatMessages.appendChild(messageDiv);
        this.scrollToBottom();
    }

    addBotTypingIndicator() {
        const typingDiv = document.createElement('div');
        typingDiv.className = 'bot-typing';
        typingDiv.id = 'typing-indicator';
        typingDiv.innerHTML = `
            <div class="loading"></div>
            <span>Thinking...</span>
        `;

        this.chatMessages.appendChild(typingDiv);
        this.scrollToBottom();

        return typingDiv;
    }

    removeTypingIndicator(typingIndicator = null) {
        if (typingIndicator) {
            typingIndicator.remove();
        } else {
            const indicator = document.getElementById('typing-indicator');
            if (indicator) {
                indicator.remove();
            }
        }
    }

    formatMessageContent(content) {
        // Simple formatting to handle newlines and basic markdown
        let formatted = this.escapeHtml(content);
        formatted = formatted.replace(/\n/g, '<br>');
        return formatted;
    }

    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }

    scrollToBottom() {
        this.chatMessages.scrollTop = this.chatMessages.scrollHeight;
    }

    // Initialize with a welcome message
    init() {
        this.addBotMessage('Hello! I\'m your Physical AI book assistant. You can ask me questions about the book content, and I\'ll provide answers based on the book\'s text. Select text on the page to ask questions specifically about that content.');
    }
}

// Initialize the chat interface when the page loads
document.addEventListener('DOMContentLoaded', () => {
    const chatInterface = new BookChatInterface();
    chatInterface.init();
});

// Export for potential use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = BookChatInterface;
}