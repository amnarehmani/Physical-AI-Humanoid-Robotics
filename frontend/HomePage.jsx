import React from 'react';
import FloatingChatbot from './src/FloatingChatbot'; // Adjust the path based on your project structure

const HomePage = () => {
  return (
    <div style={{ minHeight: '100vh', position: 'relative' }}>
      {/* Your existing page content */}
      <header>
        <h1>Welcome to Our Website</h1>
        <nav>
          <ul>
            <li>Home</li>
            <li>About</li>
            <li>Contact</li>
          </ul>
        </nav>
      </header>
      
      <main>
        <section>
          <h2>Main Content</h2>
          <p>This is the main content area of your website.</p>
          <p>The floating chatbot will appear on the right side of the screen.</p>
        </section>
      </main>
      
      <footer>
        <p>© 2025 Your Company. All rights reserved.</p>
      </footer>
      
      {/* Floating Chatbot - Will appear on all pages where this component is used */}
      <FloatingChatbot />
    </div>
  );
};

export default HomePage;