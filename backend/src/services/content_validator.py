import re
import html
from typing import Optional
import logging
from bleach import clean

from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class ContentValidator:
    """
    Service class to validate and sanitize content during ingestion
    """
    
    def __init__(self):
        # Allowed HTML tags if needed (for Markdown rendering)
        self.allowed_tags = [
            'p', 'br', 'strong', 'em', 'u', 'ol', 'ul', 'li', 
            'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'blockquote', 
            'code', 'pre', 'a', 'img'
        ]
        
        # Allowed HTML attributes
        self.allowed_attributes = {
            'a': ['href', 'title'],
            'img': ['src', 'alt', 'title'],
            'code': ['class']
        }
    
    def validate_format(self, content: str, format_type: str = "markdown") -> bool:
        """
        Validate that the content is in the expected format
        
        Args:
            content: Content to validate
            format_type: Expected format ('markdown', 'mdx', etc.)
            
        Returns:
            True if valid, False otherwise
        """
        if not content or len(content.strip()) == 0:
            logger.warning("Content is empty or contains only whitespace")
            return False
        
        # For markdown/MDX, check for basic structure
        if format_type in ['markdown', 'mdx']:
            # Check if content has reasonable structure (not just one long line)
            lines = content.split('\n')
            if len(lines) == 1 and len(lines[0]) > 2000:
                logger.warning("Content appears to be one long line, possible format issue")
                return False
        
        return True
    
    def validate_size(self, content: str, max_size: int = 1000000) -> bool:  # 1MB default
        """
        Validate that the content size is within acceptable limits
        
        Args:
            content: Content to validate
            max_size: Maximum allowed size in bytes
            
        Returns:
            True if within limits, False otherwise
        """
        content_bytes = len(content.encode('utf-8'))
        if content_bytes > max_size:
            logger.warning(f"Content size {content_bytes} exceeds maximum {max_size} bytes")
            return False
        
        return True
    
    def sanitize_content(self, content: str) -> str:
        """
        Sanitize the content to remove potentially harmful elements
        
        Args:
            content: Content to sanitize
            
        Returns:
            Sanitized content
        """
        try:
            # First, apply HTML sanitization if needed
            sanitized = clean(
                content,
                tags=self.allowed_tags,
                attributes=self.allowed_attributes,
                strip=True
            )
            
            # Unescape any HTML entities that were encoded
            sanitized = html.unescape(sanitized)
            
            # Remove any potential script tags that might have slipped through
            sanitized = re.sub(r'<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>', '', sanitized, flags=re.IGNORECASE)
            sanitized = re.sub(r'<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>', '', sanitized, flags=re.IGNORECASE)
            
            logger.info("Content sanitized successfully")
            return sanitized
            
        except Exception as e:
            logger.error(f"Error during content sanitization: {str(e)}")
            # If sanitization fails, return the original content
            return content


# Global instance of the ContentValidator
content_validator = ContentValidator()