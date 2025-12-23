import re
from typing import List, Tuple
from uuid import uuid4


class Chunker:
    """
    Service class to handle deterministic chunking of document content
    """
    
    def __init__(self, max_chunk_size: int = 1000, overlap: int = 100):
        """
        Initialize the chunker with specified parameters
        
        Args:
            max_chunk_size: Maximum size of each chunk in characters
            overlap: Number of overlapping characters between chunks
        """
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap
    
    def chunk_document(self, content: str, source_path: str) -> List[dict]:
        """
        Deterministically chunk the document content preserving semantic meaning
        
        Args:
            content: The document content to chunk
            source_path: The source path of the document for metadata
            
        Returns:
            List of chunk dictionaries with content and metadata
        """
        # First, try to split on major semantic boundaries
        # 1. Split by headers
        # 2. Then by paragraphs
        # 3. Finally by sentences if needed
        
        # Split content by headers (Markdown headers)
        header_split_pattern = r'(\n#{1,6}\s.*?\n)'
        header_parts = re.split(header_split_pattern, content)
        
        chunks = []
        chunk_id = 0
        current_section_title = ""
        
        i = 0
        while i < len(header_parts):
            part = header_parts[i].strip()
            
            # Check if this part is a header
            if part and re.match(r'\n#{1,6}\s.*?\n', '\n' + part + '\n'):
                # This is a header, update the section title
                current_section_title = part.strip().replace('#', '').strip()
                i += 1
                continue
            
            # Process content that follows the header
            if part:
                # Chunk the content
                sub_chunks = self._chunk_text(part, current_section_title)
                
                # Add chunk_order and chunk_id to each chunk
                for sub_chunk in sub_chunks:
                    chunks.append({
                        "chunk_id": str(uuid4()),  # Use UUID string for Qdrant compatibility
                        "content": sub_chunk,
                        "section_title": current_section_title,
                        "source_path": source_path,
                        "chunk_order": chunk_id
                    })
                    chunk_id += 1
            
            i += 1
        
        # If the document has no headers, chunk the entire content
        if not chunks:
            simple_chunks = self._chunk_text(content, "Introduction")
            for i, chunk in enumerate(simple_chunks):
                chunks.append({
                    "chunk_id": str(uuid4()),  # Use UUID string for Qdrant compatibility
                    "content": chunk,
                    "section_title": "Introduction",
                    "source_path": source_path,
                    "chunk_order": i
                })
        
        return chunks
    
    def _chunk_text(self, text: str, section_title: str) -> List[str]:
        """
        Helper method to chunk text content into appropriately sized pieces
        
        Args:
            text: Text content to chunk
            section_title: The section title this text belongs to
            
        Returns:
            List of text chunks
        """
        chunks = []
        current_chunk = ""
        
        # Split by paragraphs first
        paragraphs = text.split('\n\n')
        
        for paragraph in paragraphs:
            # If adding this paragraph would exceed the chunk size
            if len(current_chunk) + len(paragraph) > self.max_chunk_size:
                # If current chunk is substantial, save it
                if len(current_chunk) > self.max_chunk_size // 2:
                    chunks.append(current_chunk)
                    current_chunk = ""
                
                # If paragraph is larger than max size, split it further
                if len(paragraph) > self.max_chunk_size:
                    # Split paragraph by sentences
                    sentences = re.split(r'(?<=[.!?]) +', paragraph)
                    temp_chunk = ""
                    
                    for sentence in sentences:
                        if len(temp_chunk) + len(sentence) <= self.max_chunk_size:
                            temp_chunk += sentence + " "
                        else:
                            if temp_chunk:
                                chunks.append(temp_chunk.strip())
                            temp_chunk = sentence + " "
                    
                    if temp_chunk:
                        current_chunk = temp_chunk.strip()
                else:
                    current_chunk = paragraph
            else:
                current_chunk += "\n\n" + paragraph
        
        # Add any remaining content as a chunk
        if current_chunk.strip():
            chunks.append(current_chunk.strip())
        
        # Ensure all chunks are within size limits (handle edge cases)
        final_chunks = []
        for chunk in chunks:
            if len(chunk) > self.max_chunk_size:
                # Further subdivide if needed
                sub_chunks = self._further_subdivide(chunk)
                final_chunks.extend(sub_chunks)
            else:
                final_chunks.append(chunk)
        
        return final_chunks
    
    def _further_subdivide(self, text: str) -> List[str]:
        """
        Further subdivide text if it exceeds the maximum chunk size
        
        Args:
            text: Text to subdivide
            
        Returns:
            List of smaller text chunks
        """
        chunks = []
        start = 0
        
        while start < len(text):
            end = start + self.max_chunk_size
            
            # Try to break at a word boundary if possible
            if end < len(text):
                # Look for a space to break at
                while end > start and text[end] != ' ' and end > start + self.max_chunk_size - 200:
                    end -= 1
                
                # If we couldn't find a good break point, force break
                if end == start + self.max_chunk_size - self.max_chunk_size + 200:  # Meaning we didn't find a space
                    end = start + self.max_chunk_size
            
            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)
            
            # Move start to end, but with overlap
            start = end
            if start < len(text) and self.overlap > 0:
                # Add overlap by going back a bit
                start = max(start - self.overlap, 0)
                # But don't overlap with the previous chunk's end
                if start <= end - self.max_chunk_size:
                    start = end
        
        return chunks


# Global instance of the chunker
default_chunker = Chunker()