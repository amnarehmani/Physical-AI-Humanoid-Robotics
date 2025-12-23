# Chatbot Component

This React component provides a chat interface that connects to a FastAPI RAG backend.

## Features

- Clean, responsive UI with distinct user and bot message styling
- Support for both global book queries (`/query`) and selection-based queries (`/query/selection`)
- Loading indicators during API requests
- Error handling for backend connectivity issues
- Option to display retrieved chunk IDs for transparency
- Keyboard-friendly (Enter to submit)
- Auto-scrolling to latest messages

## Backend Endpoints Required

- `POST /query` - For global book QA
- `POST /query/selection` - For selected-text QA

Expected request format:
- For `/query`: `{ "question": "your question here" }`
- For `/query/selection`: `{ "question": "your question here", "selected_text": "text selected by user" }`

Expected response format:
- Response should contain: `{ "answer": "response text", "retrieved_chunks": [...] }` or `{ "response": "response text", "chunks": [...] }`

## Usage

### In Docusaurus Pages/MDX

```jsx
import Chatbot from '@site/src/components/Chatbot';

// Then use in your JSX:
<Chatbot />
```

### Styling

The component comes with its own CSS file (`Chatbot.css`) that provides a clean, modern chat interface. You can customize the styling by modifying this file.

### Configuration

The component behavior can be adjusted via the `chatbot-config.js` file:

- `BACKEND_URL`: The URL of your FastAPI backend
- `REQUEST_TIMEOUT`: Request timeout in milliseconds
- `ENDPOINTS`: API endpoint paths

## Development Notes

During development with Docusaurus:
- The proxy configuration in `docusaurus.config.js` forwards API requests to the backend
- The component adapts to use relative URLs during development and absolute URLs in production

For production builds, ensure your backend supports CORS requests from your domain.

## Requirements

- React (with hooks support)
- Modern browser with fetch API support
- FastAPI backend with specified endpoints