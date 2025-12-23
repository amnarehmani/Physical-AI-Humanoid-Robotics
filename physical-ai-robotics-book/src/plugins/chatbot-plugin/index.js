const path = require('path');

// Docusaurus plugin to inject the floating chatbot
module.exports = function chatbotPlugin(context, options) {
  return {
    name: 'chatbot-inject-plugin',

    // Inject the chatbot container element and script
    injectHtmlTags() {
      return {
        // Inject the chatbot container and styles in the head
        headTags: [
          {
            tagName: 'style',
            innerHTML: `
              .floating-chatbot {
                position: fixed;
                bottom: 20px;
                right: 20px;
                z-index: 10000;
                font-family: Arial, sans-serif;
              }

              .chatbot-toggle-button {
                background-color: #007bff;
                color: white;
                border: none;
                border-radius: 50%;
                width: 60px;
                height: 60px;
                font-size: 24px;
                cursor: pointer;
                box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
                display: flex;
                align-items: center;
                justify-content: center;
                transition: all 0.3s ease;
              }

              .chatbot-toggle-button:hover {
                background-color: #0056b3;
                transform: scale(1.05);
              }

              .chatbot-widget {
                width: 380px;
                height: 500px;
                position: absolute;
                bottom: 70px;
                right: 0;
                box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
                border-radius: 8px;
                overflow: hidden;
                display: flex;
                flex-direction: column;
                background: white;
                opacity: 0;
                transform: translateY(20px);
                transition: all 0.3s ease;
              }

              .chatbot-widget.open {
                opacity: 1;
                transform: translateY(0);
              }

              .chatbot-header {
                background-color: #f8f9fa;
                padding: 12px;
                border-bottom: 1px solid #dee2e6;
                display: flex;
                justify-content: space-between;
                align-items: center;
              }

              .chatbot-header h3 {
                margin: 0;
                font-size: 16px;
                color: #333;
              }

              .close-button {
                background: none;
                border: none;
                font-size: 20px;
                cursor: pointer;
                color: #6c757d;
                padding: 0;
                width: 30px;
                height: 30px;
                display: flex;
                align-items: center;
                justify-content: center;
                border-radius: 50%;
              }

              .close-button:hover {
                background-color: #e9ecef;
              }

              /* Responsive adjustments */
              @media (max-width: 768px) {
                .chatbot-widget {
                  width: calc(100vw - 40px);
                  height: 60vh;
                  left: 20px;
                  right: 20px;
                  bottom: 80px;
                }
              }
            `,
          }
        ],
        // Inject the chatbot container and initialization script in the body
        preBodyTags: [
          {
            tagName: 'div',
            attributes: {
              id: 'floating-chatbot-container',
              style: 'display: none;' // Initially hidden until JS loads
            },
          },
        ],
        postBodyTags: [
          {
            tagName: 'script',
            innerHTML: `
              (function() {
                // Wait for DOM to be ready
                if (document.readyState === 'loading') {
                  document.addEventListener('DOMContentLoaded', initializeChatbot);
                } else {
                  initializeChatbot();
                }
                
                function initializeChatbot() {
                  // Create the chatbot container
                  const chatbotContainer = document.createElement('div');
                  chatbotContainer.className = 'floating-chatbot';
                  chatbotContainer.id = 'docusaurus-chatbot';
                  
                  // Create the toggle button
                  const toggleButton = document.createElement('button');
                  toggleButton.className = 'chatbot-toggle-button';
                  toggleButton.setAttribute('aria-label', 'Open chatbot');
                  toggleButton.innerHTML = '💬';
                  
                  // Create the chatbot widget
                  const chatbotWidget = document.createElement('div');
                  chatbotWidget.className = 'chatbot-widget';
                  chatbotWidget.innerHTML = \`
                    <div className="chatbot-header">
                      <h3>Book Q&A Chatbot</h3>
                      <button className="close-button" aria-label="Close chatbot">×</button>
                    </div>
                    <div style="flex: 1; overflow: hidden; height: 400px;">
                      <div id="chatbot-content">Loading chatbot...</div>
                    </div>
                  \`;
                  
                  // Create the chatbot content area
                  const chatbotContent = document.createElement('div');
                  chatbotContent.id = 'chatbot-content';
                  chatbotContent.style.height = '100%';
                  chatbotContent.innerHTML = 'Loading chatbot...';
                  
                  // Append content to widget
                  const header = document.createElement('div');
                  header.className = 'chatbot-header';
                  header.innerHTML = '<h3>Book Q&A Chatbot</h3><button class="close-button" aria-label="Close chatbot">×</button>';
                  
                  chatbotWidget.appendChild(header);
                  chatbotWidget.appendChild(chatbotContent);
                  
                  // Add widget to container but hide initially
                  chatbotContainer.appendChild(chatbotWidget);
                  chatbotContainer.appendChild(toggleButton);
                  
                  // Add to the page
                  document.body.appendChild(chatbotContainer);
                  
                  // Set up event handlers
                  let isOpen = false;
                  
                  toggleButton.addEventListener('click', function() {
                    isOpen = !isOpen;
                    chatbotWidget.className = isOpen ? 'chatbot-widget open' : 'chatbot-widget';
                    toggleButton.innerHTML = isOpen ? '×' : '💬';
                    toggleButton.setAttribute('aria-label', isOpen ? 'Close chatbot' : 'Open chatbot');
                    
                    // Load the React component dynamically if not already loaded
                    if (isOpen && !window.chatbotLoaded) {
                      loadReactChatbot();
                    }
                  });
                  
                  // Close button functionality
                  const closeButton = chatbotWidget.querySelector('.close-button');
                  closeButton.addEventListener('click', function() {
                    isOpen = false;
                    chatbotWidget.className = 'chatbot-widget';
                    toggleButton.innerHTML = '💬';
                    toggleButton.setAttribute('aria-label', 'Open chatbot');
                  });
                  
                  // Load the React chatbot component
                  function loadReactChatbot() {
                    // Check if React and ReactDOM are available
                    if (typeof window !== 'undefined' && window.React && window.ReactDOM) {
                      // Dynamically load the chatbot component via React
                      const script = document.createElement('script');
                      script.src = '/js/chatbot-bundle.js';  // This would be built separately
                      script.async = true;
                      document.head.appendChild(script);
                      window.chatbotLoaded = true;
                    } else {
                      chatbotContent.innerHTML = 'Error: React not available';
                    }
                  }
                }
              })();
            `,
          }
        ],
      };
    },
  };
};