import React, { useState, useRef, useEffect } from 'react';
import './chat-interface.css'; // Import the CSS file
import CONFIG from './chatbot-config.js';

const Chatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    // Add user message to chat
    const userMessage = { text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    
    const question = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      // Determine which endpoint to use based on whether there's selected text
      const endpoint = selectedText ? '/query/selection' : '/query';
      const requestBody = selectedText 
        ? { question, selected_text: selectedText }
        : { question };

      // Determine the correct URL based on environment
      // During development with Docusaurus proxy, use relative paths
      // For production, use the configured backend URL
      const isDev = process.env.NODE_ENV !== 'production';
      let apiUrl = endpoint;

      // In production or when not running via Docusaurus dev server,
      // use the configured backend URL
      if (!isDev || typeof window === 'undefined') {
        apiUrl = `${CONFIG.BACKEND_URL}${endpoint}`;
      }

      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.status}`);
      }

      const data = await response.json();
      
      // Add bot response to chat
      const botMessage = {
        text: data.response || data.answer || 'No response from backend',
        sender: 'bot',
        chunks: data.chunks || data.retrieved_chunks || [] // Optional: display retrieved chunk IDs
      };
      
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error fetching response:', error);
      const errorMessage = {
        text: `Error: Unable to reach backend (${error.message})`,
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const handleTextSelection = () => {
    const selected = window.getSelection().toString().trim();
    if (selected) {
      setSelectedText(selected);
      // Optionally notify the user that text has been selected
      console.log('Selected text for RAG:', selected.substring(0, 50) + (selected.length > 50 ? '...' : ''));
    }
  };

  // Allow users to clear the selected text if needed
  const clearSelectedText = () => {
    setSelectedText('');
  };

  return (
    <div className="chatbot-container">
      <div className="chat-header">
        <h3>Book Q&A Chatbot</h3>
        {selectedText && (
          <div className="selected-text-indicator">
            Selected text active: "{selectedText.substring(0, 50)}..."
            <button onClick={clearSelectedText} className="clear-btn">Clear</button>
          </div>
        )}
      </div>
      
      <div className="chat-messages" onMouseUp={handleTextSelection}>
        {messages.map((msg, index) => (
          <div 
            key={index} 
            className={`message ${msg.sender}-message`}
            onMouseUp={handleTextSelection} // Allow selecting text in messages too
          >
            <div className="message-text">{msg.text}</div>
            
            {/* Optional: Show retrieved chunks */}
            {msg.chunks && msg.chunks.length > 0 && (
              <div className="chunks-info">
                <details>
                  <summary>Retrieved chunks:</summary>
                  <ul>
                    {msg.chunks.map((chunk, idx) => (
                      <li key={idx}>{chunk.id || chunk.chunk_id || chunk}</li>
                    ))}
                  </ul>
                </details>
              </div>
            )}
          </div>
        ))}
        
        {isLoading && (
          <div className="message bot-message">
            <div className="loading-indicator">Thinking...</div>
          </div>
        )}
        
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="chat-input-form">
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the book..."
          className="chat-input"
          rows="3"
        />
        <button type="submit" disabled={isLoading} className="send-button">
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default Chatbot;