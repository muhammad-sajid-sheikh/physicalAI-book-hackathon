# Data Model: Qdrant Retrieval System

**Feature**: 001-qdrant-retrieval
**Date**: 2025-12-28

## Entity: Query
**Description**: User input for retrieval operation

**Attributes**:
- `text_content` (string): The text query to search for
- `top_k_count` (integer): Number of results to return (default: 5)
- `filters` (dict, optional): Additional filters for the search

**Validation Rules**:
- `text_content` must not be empty or whitespace-only
- `top_k_count` must be between 1 and 100

## Entity: RetrievedChunk
**Description**: Individual text chunk returned as search result

**Attributes**:
- `content` (string): The text content of the chunk
- `source_url` (string): URL of the original document
- `document_title` (string): Title of the original document
- `chunk_id` (string): Unique identifier for this chunk
- `document_id` (string): Identifier for the parent document
- `chunk_number` (integer): Position of this chunk in the document
- `similarity_score` (float): Similarity score between 0 and 1
- `token_count` (integer): Number of tokens in the chunk
- `model_used` (string): Embedding model used for this chunk
- `metadata` (dict): Additional metadata associated with the chunk

**Validation Rules**:
- `content` must not be empty
- `source_url` must be a valid URL format
- `similarity_score` must be between 0 and 1
- `chunk_number` must be non-negative

## Entity: QdrantConnection
**Description**: Configuration and state for Qdrant database connection

**Attributes**:
- `url` (string): Qdrant instance URL
- `api_key` (string): Authentication API key
- `collection_name` (string): Name of the collection to search
- `connection_status` (string): Current status (connected/disconnected/error)

**Validation Rules**:
- `url` must be a valid URL format
- `api_key` must not be empty
- `collection_name` must not be empty

## Entity: SearchResult
**Description**: Container for search results

**Attributes**:
- `query` (string): The original query text
- `total_results` (integer): Total number of results returned
- `retrieved_chunks` (list[RetrievedChunk]): List of retrieved chunks
- `execution_time` (float): Time taken to execute the search in seconds
- `search_metadata` (dict): Additional search metadata

**Validation Rules**:
- `total_results` must match the length of `retrieved_chunks`
- `execution_time` must be non-negative

## Entity: ValidationResult
**Description**: Result of validation operation

**Attributes**:
- `query` (string): The query that was validated
- `total_results` (integer): Total number of results to validate
- `results_validated` (integer): Number of results that passed validation
- `metadata_validated` (integer): Number of results with valid metadata
- `content_validated` (integer): Number of results with valid content
- `validation_details` (list[dict]): Detailed validation results for each result

**Validation Rules**:
- `results_validated` must not exceed `total_results`
- `metadata_validated` must not exceed `total_results`
- `content_validated` must not exceed `total_results`

## Relationships

- `Query` → `SearchResult` (one-to-many): One query produces one search result
- `SearchResult` → `RetrievedChunk` (one-to-many): One search result contains multiple chunks
- `QdrantConnection` → `SearchResult` (one-to-many): One connection can produce multiple search results