import React, { useState, useRef, useEffect } from 'react';
import chatbotConfig from './chatbot-config.js';
import './chatbot.css';

const FloatingChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [isChatOpen, setIsChatOpen] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize session ID
  useEffect(() => {
    if (!sessionId) {
      setSessionId(`session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`);
    }
  }, [sessionId]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { text: inputValue, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Use the correct endpoint path from config
      const response = await fetch(`${chatbotConfig.backendUrl}${chatbotConfig.endpoints.query}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputValue,
          session_id: sessionId  // Include session_id as required
        })
      });

      // Check if response is OK before parsing
      if (!response.ok) {
        // Handle different error statuses
        if (response.status === 404) {
          throw new Error('Backend endpoint not found (404). Check backend URL and endpoint path.');
        } else if (response.status === 500) {
          throw new Error('Backend server error (500). Check backend logs.');
        } else {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
      }

      const data = await response.json();

      // Create bot message with the response
      const botMessage = {
        text: data.answer || 'Sorry, I could not process your request.',
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        text: `Error: ${error.message || 'Sorry, there was an error processing your request. Please try again.'}`,
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
      {/* Floating toggle button - always visible when chat is closed */}
      { !isChatOpen && (
        <button
          className="chat-toggle-btn"
          onClick={() => setIsChatOpen(true)}
        >
          💬
        </button>
      )}

      {/* Chat container - fixed to the right side */}
      { isChatOpen && (
        <div className="floating-chatbot">
          <div className="chat-container">
            <div className="chat-header">
              <h3>AI Book Assistant</h3>
              <button className="close-btn" onClick={() => setIsChatOpen(false)}>×</button>
            </div>
            <div className="chat-messages">
              {messages.map((msg, index) => (
                <div key={index} className={`message ${msg.sender}`}>
                  <div className="message-text">{msg.text}</div>
                  <div className="message-timestamp">{msg.timestamp.toLocaleTimeString()}</div>
                </div>
              ))}
              {isLoading && (
                <div className="message bot">
                  <div className="message-text">Thinking...</div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
            <div className="chat-input-area">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the book..."
                disabled={isLoading}
                rows="3"
              />
              <button
                onClick={handleSendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="send-btn"
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;