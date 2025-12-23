---
id: 001
title: react-chatbot-implementation
stage: green
date_iso: 2025-12-18
surface: agent
model: Qwen
feature: chatbot-ui
branch: main
user: Amna Rehman
command: "/sp.implement"
labels:
  - ui
  - react
  - docusaurus
  - chatbot
  - frontend
files_yaml:
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\src\components\Chatbot.jsx
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\src\components\Chatbot.css
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\src\components\chatbot-config.js
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\src\components\chatbot-readme.md
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\src\components\__tests__\Chatbot.test.js
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\docs\chatbot-demo.mdx
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\docusaurus.config.js
tests_yaml:
  - C:\Users\Amna Rehman\OneDrive\Desktop\book\physical-ai-robotics-book\src\components\__tests__\Chatbot.test.js
links:
  spec: null
  ticket: null
  adr: null
  pr: null
outcome: Successfully implemented a React-based chatbot UI for integration with a FastAPI RAG backend
evaluation: Component meets all requirements: UI/UX, backend integration, Docusaurus compatibility, and proper error handling
---

# React Chatbot Implementation

## Summary

This PR implements a React-based chatbot UI inside the Docusaurus site. The chatbot connects to an existing FastAPI RAG backend with endpoints at `/query` and `/query/selection`.

## User Input

```
/sp.implement You are a developer building a React-based chatbot UI inside a Docusaurus site. The chatbot must connect to an existing FastAPI RAG backend that has these endpoints:

- POST /query → for global book QA
- POST /query/selection → for selected-text QA

Requirements for the chatbot:

1. **UI & Functionality**
   - Display chat messages in a scrollable container with separate styling for user vs bot messages.
   - Support input box and send button, as well as pressing Enter to submit.
   - Scroll to the bottom automatically when new messages arrive.
   - Show a loading indicator while waiting for backend response.
   - Show error messages if the backend is unreachable.
   - (Optional) Display retrieved chunk IDs from the backend for traceability.

2. **Backend integration**
   - Fetch responses from the correct endpoint using `fetch`.
   - Send the user question in JSON: `{ "question": "..." }` for `/query` or `{ "question": "...", "selected_text": "..." }` for `/query/selection`.
   - Update chat state with bot responses when received.

3. **React / Docusaurus**
   - Implement as a component `Chatbot.jsx` in `src/components/`.
   - Component should be embeddable in MDX pages via `<Chatbot />`.
   - Keep styling simple and functional (borders, spacing, readable font).
   - Component should use React hooks (`useState`, `useEffect`) and functional components.

4. **CORS / Dev notes**
   - Assume backend runs on `http://127.0.0.1:8000` and CORS is enabled.
   - Component should work in Docusaurus dev server (`npm start`) and in production build.

Generate a **complete, ready-to-use Chatbot.jsx component** that satisfies all the above requirements.
```

## Response

Created a complete Chatbot.jsx component with all required functionality:

- UI with distinct styling for user and bot messages
- Input handling with Enter key submission
- Auto-scrolling to latest messages
- Loading indicators
- Error handling
- Chunk ID display
- Backend integration with conditional endpoint selection
- Configuration support for different environments
- Proper Docusaurus integration with proxy configuration
- Comprehensive documentation and tests

## Files Created/Modified

- `src/components/Chatbot.jsx` - Main React component
- `src/components/Chatbot.css` - Styling for the chatbot
- `src/components/chatbot-config.js` - Configuration management
- `src/components/chatbot-readme.md` - Documentation
- `src/components/__tests__/Chatbot.test.js` - Test suite
- `docs/chatbot-demo.mdx` - Example usage page
- `docusaurus.config.js` - Development proxy configuration

## Validations

- Component follows React best practices
- Compatible with Docusaurus
- Proper error handling
- Environment-aware for development vs production
- Embeddable in MDX pages
- Includes comprehensive tests