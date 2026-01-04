# Quickstart Guide: Qdrant Retrieval System

**Feature**: 001-qdrant-retrieval
**Date**: 2025-12-28

## Overview

This guide provides instructions for setting up and using the Qdrant retrieval system to perform semantic search on stored book content vectors.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to Qdrant Cloud instance with existing vectors
- Cohere API key for query embedding
- Existing vector collection in Qdrant (from spec1)

## Setup

### 1. Clone or navigate to the project directory

```bash
cd C:\Users\DELL\physical-ai-humanoid-robotics-book\backend
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
# Or install required packages directly:
pip install qdrant-client cohere python-dotenv requests beautifulsoup4
```

### 3. Configure environment variables

Create or update the `.env` file with your credentials:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=Rag-chatbot
```

## Usage

### Running the retrieval system

```bash
python retrieve.py
```

This will run a demonstration of the retrieval system with test queries.

### Using the retrieval system programmatically

```python
from retrieval import QdrantRetriever

# Initialize the retriever
retriever = QdrantRetriever()

# Perform a search
results = retriever.retrieve_content("What is cognitive planning?", top_k=3)

# Process results
for result in results:
    print(f"Score: {result['similarity_score']}")
    print(f"Content: {result['content'][:100]}...")
    print(f"Source: {result['source_url']}")
    print("---")
```

### Validating retrieval results

```python
# Validate a query result
validation = retriever.validate_retrieval("What is cognitive planning?", top_k=2)
print(f"Valid results: {validation['results_validated']}/{validation['total_results']}")
```

## Configuration

The system uses configuration from the `.env` file and can be adjusted with these parameters:

- `CHUNK_SIZE_TOKENS`: Size of text chunks (default: 512)
- `OVERLAP_PERCENTAGE`: Overlap between chunks (default: 0.2)
- `BATCH_SIZE`: Batch size for API calls (default: 100)
- `TIMEOUT`: Request timeout in seconds (default: 30)

## Troubleshooting

### Common Issues

1. **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY in `.env` file
2. **Authentication errors**: Check that API keys are correctly configured
3. **No results**: Ensure the Qdrant collection contains vectors from the ingestion pipeline
4. **Embedding errors**: Verify COHERE_API_KEY is valid and has proper permissions

### Testing Connection

To test the Qdrant connection:

```python
from src.storage.qdrant_client import QdrantStorage

storage = QdrantStorage()
info = storage.get_collection_info()
print(f"Collection: {info['collection_name']}")
print(f"Points: {info['point_count']}")
```

## Next Steps

- Customize query parameters for your specific use case
- Implement additional validation checks
- Add error handling for production use
- Integrate with your application's search interface