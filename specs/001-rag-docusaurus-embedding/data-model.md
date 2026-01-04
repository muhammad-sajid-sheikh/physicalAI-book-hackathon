# Data Model: URL Ingestion and Embedding Pipeline

**Feature**: URL Ingestion and Embedding Pipeline | **Branch**: `001-rag-docusaurus-embedding` | **Date**: 2025-12-28
**Purpose**: Document the data structures and entities for the ingestion pipeline

## Overview

This document describes the data models and structures for the URL ingestion and embedding pipeline that fetches content from Docusaurus documentation sites, processes and chunks the text, generates embeddings using Cohere models, and stores them with metadata in Qdrant vector database.

## Core Entities

### 1. DocumentContent

The primary entity representing extracted content from a URL.

**Structure**:
- `source_url`: String (URL of the source document)
- `title`: String (title of the document)
- `content`: String (clean text content extracted from the page)
- `html_content`: String (original HTML content, optional)
- `metadata`: Dictionary (additional metadata like author, date, etc.)
- `created_at`: DateTime (timestamp of extraction)
- `processed_at`: DateTime (timestamp of processing completion)

**Relationships**: One-to-many with TextChunk (one document can have multiple chunks)

### 2. TextChunk

Represents a segmented portion of document content ready for embedding.

**Structure**:
- `chunk_id`: String (unique identifier for the chunk)
- `document_id`: String (reference to parent document)
- `content`: String (the actual text content of the chunk)
- `start_offset`: Integer (character position where chunk starts in original document)
- `end_offset`: Integer (character position where chunk ends in original document)
- `chunk_number`: Integer (sequence number of the chunk in the document)
- `token_count`: Integer (number of tokens in the chunk)
- `semantic_boundary`: String (type of boundary where chunk was split)

**Relationships**: Many-to-one with DocumentContent (many chunks belong to one document)

### 3. EmbeddingVector

Represents the vector embedding of a text chunk.

**Structure**:
- `embedding_id`: String (unique identifier for the embedding)
- `chunk_id`: String (reference to the source chunk)
- `vector`: Array of Float (the actual embedding vector, 1024-dimensional for Cohere v3)
- `model_used`: String (name of the embedding model used)
- `created_at`: DateTime (timestamp of embedding generation)
- `embedding_metadata`: Dictionary (additional metadata about the embedding)

**Relationships**: One-to-one with TextChunk (each chunk has one embedding)

### 4. VectorRecord

The entity stored in Qdrant representing a complete searchable record.

**Structure**:
- `record_id`: String (unique identifier for the Qdrant record)
- `vector`: Array of Float (embedding vector)
- `payload`: Dictionary (metadata stored with the vector)
  - `source_url`: String (original URL)
  - `document_title`: String (title of the source document)
  - `chunk_content`: String (the text content of the chunk)
  - `chunk_id`: String (reference to the source chunk)
  - `document_id`: String (reference to the source document)
  - `chunk_number`: Integer (position in document)
  - `created_at`: DateTime (timestamp)
  - `metadata`: Dictionary (additional custom metadata)

**Relationships**: One-to-one with EmbeddingVector (each embedding becomes one Qdrant record)

### 5. ProcessingConfig

Configuration parameters for the ingestion pipeline.

**Structure**:
- `cohere_api_key`: String (API key for Cohere services)
- `qdrant_url`: String (URL for Qdrant instance)
- `qdrant_api_key`: String (API key for Qdrant)
- `qdrant_collection_name`: String (name of the target collection)
- `chunk_size_tokens`: Integer (target size for text chunks in tokens)
- `chunk_overlap_percentage`: Float (percentage overlap between chunks)
- `batch_size`: Integer (number of chunks to process in each API batch)
- `rate_limit_delay`: Float (delay between API calls to respect rate limits)
- `max_retries`: Integer (maximum number of retry attempts for failed operations)

## Data Flow Relationships

### Ingestion Pipeline Flow:
1. **URL Input** → DocumentContent (extracted via web scraping)
2. **DocumentContent** → TextChunk (via chunking algorithm)
3. **TextChunk** → EmbeddingVector (via Cohere API)
4. **EmbeddingVector** → VectorRecord (with metadata enrichment)
5. **VectorRecord** → Qdrant Storage

### Metadata Propagation:
- Source URL flows from DocumentContent → TextChunk → EmbeddingVector → VectorRecord
- Document title flows through the same chain
- Chunk positioning information (start/end offsets, chunk number) flows from DocumentContent → TextChunk → VectorRecord

## Qdrant Schema Design

### Collection Configuration:
- **Vector Size**: 1024 (for Cohere embed-english-v3.0 model)
- **Distance Metric**: Cosine (optimal for semantic search)
- **Collection Name**: Configurable via ProcessingConfig

### Payload Structure:
The payload in Qdrant will contain rich metadata to enable filtering and retrieval:

```json
{
  "source_url": "https://example.com/docs/intro",
  "document_title": "Introduction to Docusaurus",
  "chunk_content": "This is the actual text content...",
  "chunk_id": "chunk-12345",
  "document_id": "doc-67890",
  "chunk_number": 1,
  "created_at": "2025-12-28T10:00:00Z",
  "metadata": {
    "word_count": 120,
    "language": "en",
    "source_type": "docusaurus",
    "processing_version": "1.0"
  }
}
```

## Validation Rules

### DocumentContent Validation:
- `source_url` must be a valid URL format
- `content` must not be empty
- `created_at` must be current timestamp

### TextChunk Validation:
- `token_count` must be <= target chunk size
- `start_offset` must be less than `end_offset`
- `chunk_number` must be >= 0

### EmbeddingVector Validation:
- `vector` must have exactly 1024 elements for Cohere v3
- `vector` elements must be valid floats
- `model_used` must match configured model

## State Transitions

### Processing States:
- `PENDING`: Document URL identified but not yet fetched
- `FETCHED`: Content extracted from URL
- `CHUNKED`: Content split into text chunks
- `EMBEDDED`: Embeddings generated for all chunks
- `STORED`: All vectors stored in Qdrant
- `FAILED`: Processing failed at any stage

Each entity tracks its processing state to enable resumable operations and error handling.