# Research Findings: RAG Chatbot Backend

## Decision 1: Selected Text Size Limits

**Issue**: Maximum size limit for selected text in the /query/selection endpoint

**Decision**: 10KB limit for selected text

**Rationale**: This allows for substantial text selections while preventing resource exhaustion. A 10KB limit is reasonable for the amount of text a user might select in a document and should fit within typical LLM context windows.

**Alternatives considered**:
- 1KB: Too restrictive for meaningful selections
- 5KB: Reasonable but potentially limiting for longer passages
- 10KB: Balanced choice that allows for substantial selections
- 50KB: Too large and could impact performance significantly

## Decision 2: Concurrent Ingest Operations

**Issue**: Handling concurrent ingest operations

**Decision**: Implement rate limiting with a maximum of 2 concurrent ingestion operations

**Rationale**: This protects system resources while still allowing for some parallelism. Ingestion is resource-intensive, so limiting concurrency prevents system overload while maintaining reasonable throughput for multiple documents.

**Alternatives considered**:
- Sequential processing: Simple but potentially slow
- Rate limiting: Balances resource usage with performance
- Full concurrent processing: Could overwhelm system resources
- Queue-based processing: More complex than needed for this use case

## Decision 3: Maximum Book Content Size

**Issue**: Expected maximum size of book content to ingest

**Decision**: Support for books up to 1000 pages with adaptive chunking

**Rationale**: This covers the majority of technical and documentation books while requiring adaptive chunking strategies for larger volumes. The system will use different chunking strategies based on document size.

**Alternatives considered**:
- Small books (<100 pages): Too limiting for comprehensive documentation
- Medium books (100-500 pages): Good for many books but not all
- Large books (500+ pages): Appropriate for comprehensive documentation
- 1000+ pages: Covers all reasonable book sizes for this use case

## Decision 4: Incremental Document Updates

**Issue**: Support for incremental updates to existing documents

**Decision**: Implement full re-ingestion approach with versioning

**Rationale**: Full re-ingestion is simpler to implement and ensures consistency. It avoids complex change detection algorithms and guarantees that the vector store and metadata are always in sync with the source document.

**Alternatives considered**:
- Full re-ingestion: Simple and reliable, chosen approach
- Incremental updates: More complex but potentially more efficient
- No updates allowed: Too restrictive for practical use
- Delta-based updates: Complex implementation with many edge cases