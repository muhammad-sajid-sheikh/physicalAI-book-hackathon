# Data Model: RAG FastAPI Integration

## Overview
This document defines the key data structures for the FastAPI integration with the RAG system.

## Entities

### QueryRequest
**Description**: A JSON object containing the user's query and optional parameters
**Fields**:
- query: String (required) - The user's question or query text
- top_k: Integer (optional) - Number of results to retrieve from knowledge base (default: 5)
- temperature: Number (optional) - Response randomness parameter (default: 0.1)

### QueryResponse
**Description**: A JSON object containing the agent's response and metadata
**Fields**:
- answer: String - The agent's response to the query
- sources: Array - List of sources used to generate the response
- success: Boolean - Whether the query was processed successfully
- error: String (optional) - Error message if processing failed
- performance: Object (optional) - Performance metrics for the request

### Source
**Description**: Information about a source used to generate the response
**Fields**:
- title: String - Title of the source document
- url: String - URL of the source
- similarity_score: Number - Relevance score of the source
- chunk_id: String - ID of the specific content chunk
- document_id: String - ID of the source document

### PerformanceMetrics
**Description**: Performance metrics for the query processing
**Fields**:
- total_duration: Number - Total time to process the request in seconds
- retrieve_duration: Number - Time spent retrieving content in seconds
- generate_duration: Number - Time spent generating response in seconds

## API Request/Response Models

### POST /api/query Request
**Structure**: QueryRequest object with validation

### POST /api/query Response
**Structure**: QueryResponse object with validation

## Validation Rules

1. **QueryRequest.query** must not be empty
2. **QueryRequest.top_k** must be between 1 and 10 if provided
3. **QueryRequest.temperature** must be between 0 and 1 if provided
4. **QueryResponse.success** must be true when no error is present
5. **QueryResponse.answer** must not be empty when success is true

## Relationships

1. **QueryRequest** is processed by **RAGAgent** to produce **QueryResponse**
2. **QueryResponse** contains multiple **Source** objects
3. **QueryResponse** may contain **PerformanceMetrics** data

## State Transitions

1. **QueryRequest** transitions from "received" → "processing" → "responded"
2. **QueryResponse** transitions from "generating" → "completed" or "failed"

## API Schema

### Request Schema (Pydantic Model)
```python
class QueryRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5
    temperature: Optional[float] = 0.1
```

### Response Schema (Pydantic Model)
```python
class Source(BaseModel):
    title: str
    url: str
    similarity_score: float
    chunk_id: str
    document_id: str

class PerformanceMetrics(BaseModel):
    total_duration: float
    retrieve_duration: float
    generate_duration: float

class QueryResponse(BaseModel):
    answer: str
    sources: List[Source]
    success: bool
    error: Optional[str] = None
    performance: Optional[PerformanceMetrics] = None
```