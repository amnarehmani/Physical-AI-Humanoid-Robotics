import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    """Configuration settings for the RAG Chatbot Backend"""
    
    # Database settings
    NEON_DATABASE_URL: str = os.getenv("NEON_DATABASE_URL", "")
    
    # Qdrant settings
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_CLUSTER_ID: str = os.getenv("QDRANT_CLUSTER_ID", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    
    # Cohere settings
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    
    # Application settings
    APP_NAME: str = os.getenv("APP_NAME", "RAG Chatbot Backend")
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = os.getenv("PROJECT_NAME", "RAG Chatbot Backend")
    
    # Ingestion settings
    MAX_SELECTED_TEXT_LENGTH: int = int(os.getenv("MAX_SELECTED_TEXT_LENGTH", "10240"))  # 10KB
    MAX_CONCURRENT_INGEST: int = int(os.getenv("MAX_CONCURRENT_INGEST", "2"))
    MAX_BOOK_PAGES: int = int(os.getenv("MAX_BOOK_PAGES", "1000"))
    
    # Query settings
    RESPONSE_TIMEOUT: int = int(os.getenv("RESPONSE_TIMEOUT", "5000"))  # 5 seconds in ms
    MAX_RETRIEVAL_RESULTS: int = int(os.getenv("MAX_RETRIEVAL_RESULTS", "10"))
    
    # Validation
    def validate_required_settings(self) -> list[str]:
        """Validate that all required environment variables are set"""
        required_vars = [
            ("NEON_DATABASE_URL", self.NEON_DATABASE_URL),
            ("QDRANT_API_KEY", self.QDRANT_API_KEY),
            ("QDRANT_CLUSTER_ID", self.QDRANT_CLUSTER_ID),
            ("QDRANT_URL", self.QDRANT_URL),
            ("COHERE_API_KEY", self.COHERE_API_KEY),
        ]
        
        errors = []
        for name, value in required_vars:
            if not value:
                errors.append(f"Missing required environment variable: {name}")
        
        return errors

# Create a global settings instance
settings = Settings()

# Validate settings at startup
validation_errors = settings.validate_required_settings()
if validation_errors:
    raise ValueError("Configuration errors:\n" + "\n".join(validation_errors))