import React, { useState, useRef, useEffect } from 'react';
import './chatbot.css';
import CONFIG from './chatbot-config';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! How can I help you today?', sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const toggleChatbot = () => setIsOpen(!isOpen);

  const handleSend = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userText = inputValue;

    setMessages(prev => [
      ...prev,
      { id: Date.now(), text: userText, sender: 'user' }
    ]);

    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(
        `${CONFIG.BACKEND_URL}${CONFIG.ENDPOINTS.QUERY}`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            question: userText,
            session_id: 'web-ui',
          }),
        }
      );

      console.log('Response status:', response.status);
      const data = await response.json();
      console.log('Response body:', data);

      if (!response.ok) {
        throw new Error(data.detail || `HTTP ${response.status}`);
      }

      setMessages(prev => [
        ...prev,
        {
          id: Date.now() + 1,
          text: data.response || data.answer || 'No response from backend',
          sender: 'bot'
        }
      ]);
    } catch (error) {
      setMessages(prev => [
        ...prev,
        {
          id: Date.now() + 1,
          text: `Error connecting to backend: ${error.message}`,
          sender: 'bot'
        }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleSend();
    }
  };

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <>
      {/* Floating Button */}
      <div className="chatbot-floating-button">
        <button onClick={toggleChatbot}>{isOpen ? '×' : '💬'}</button>
      </div>

      {/* Chat Window */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button onClick={toggleChatbot}>×</button>
          </div>

          <div className="chat-messages">
            {messages.map(msg => (
              <div
                key={msg.id}
                className={`message ${msg.sender === 'user' ? 'user-message' : 'bot-message'}`}
              >
                {msg.text}
              </div>
            ))}
            {isLoading && (
              <div className="loading-indicator">Thinking...</div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className="chat-input-form" onSubmit={e => e.preventDefault()}>
            <textarea
              className="chat-input"
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              disabled={isLoading}
            />
            <button
              type="button"
              className="send-button"
              onClick={handleSend}
              disabled={isLoading}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;
