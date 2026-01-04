# Quickstart Guide: URL Ingestion and Embedding Pipeline

**Feature**: URL Ingestion and Embedding Pipeline | **Branch**: `001-rag-docusaurus-embedding` | **Date**: 2025-12-28
**Purpose**: Quick implementation guide for the Docusaurus content ingestion pipeline

## Overview

This quickstart guide provides the essential steps to implement a URL ingestion and embedding pipeline that fetches content from Docusaurus documentation sites, processes and chunks the text, generates embeddings using Cohere models, and stores them with metadata in Qdrant vector database.

## Prerequisites

- Python 3.11+ installed
- Cohere API key
- Qdrant Cloud account and API key
- Basic understanding of Python, web scraping, and vector databases

## Setup Steps

### 1. Create the Backend Directory Structure

```bash
mkdir backend
cd backend
```

### 2. Initialize the Project with uv

Create a `pyproject.toml` file:

```toml
[project]
name = "docusaurus-ingestion-pipeline"
version = "0.1.0"
description = "Pipeline for ingesting Docusaurus content and generating embeddings"
authors = ["Your Name <your.email@example.com>"]
dependencies = [
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=4.0.0",
    "qdrant-client>=1.7.0",
    "python-dotenv>=1.0.0",
    "pytest>=7.4.0",
]
requires-python = ">=3.11"

[build-system]
requires = ["setuptools>=61.0"]
build-backend = "setuptools.build_meta"
```

### 3. Install Dependencies

```bash
uv sync
# or if using traditional pip:
pip install -r requirements.txt
```

### 4. Create the Project Structure

```
backend/
├── main.py                 # Main ingestion pipeline entry point
├── requirements.txt        # Python dependencies
├── pyproject.toml          # Project configuration for uv
├── .env.example           # Example environment variables
├── .env                  # Local environment variables (gitignored)
├── src/
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── url_fetcher.py     # URL fetching and content extraction
│   │   ├── text_cleaner.py    # Text cleaning and preprocessing
│   │   ├── chunker.py         # Text segmentation logic
│   │   └── processor.py       # Main processing orchestrator
│   ├── embedding/
│   │   ├── __init__.py
│   │   ├── generator.py       # Cohere embedding generation
│   │   └── client.py          # Cohere API client
│   └── storage/
│       ├── __init__.py
│       ├── qdrant_client.py   # Qdrant vector database client
│       └── metadata.py        # Metadata schema and handling
└── tests/
    ├── __init__.py
    ├── test_ingestion.py      # Ingestion pipeline tests
    ├── test_chunking.py       # Text chunking tests
    ├── test_embedding.py      # Embedding generation tests
    └── test_storage.py        # Storage functionality tests
```

### 5. Create Environment File

Create `.env.example`:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=docs_embeddings
```

## Implementation Steps

### 1. Main Pipeline Implementation

Create the main pipeline in `main.py`:

```python
import os
import asyncio
from dotenv import load_dotenv
from src.ingestion.processor import DocumentProcessor
from src.embedding.generator import EmbeddingGenerator
from src.storage.qdrant_client import QdrantStorage

def main():
    # Load environment variables
    load_dotenv()

    # Initialize components
    processor = DocumentProcessor()
    embedder = EmbeddingGenerator()
    storage = QdrantStorage()

    # Example URLs to process
    urls = [
        "https://example-docusaurus-site.com/docs/intro",
        "https://example-docusaurus-site.com/docs/getting-started"
    ]

    # Process each URL
    for url in urls:
        print(f"Processing: {url}")

        # Step 1: Fetch and extract content
        document = processor.fetch_and_extract(url)

        # Step 2: Chunk the content
        chunks = processor.chunk_content(document)

        # Step 3: Generate embeddings
        embeddings = embedder.generate_embeddings(chunks)

        # Step 4: Store in Qdrant
        storage.store_embeddings(embeddings)

        print(f"Completed processing: {url}")

    print("All documents processed successfully!")

if __name__ == "__main__":
    main()
```

### 2. URL Fetching Implementation

In `src/ingestion/url_fetcher.py`:

```python
import requests
from bs4 import BeautifulSoup
from typing import Dict, Any

class URLFetcher:
    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; DocusaurusBot/1.0)'
        })

    def fetch_content(self, url: str) -> Dict[str, Any]:
        """Fetch and extract content from a Docusaurus URL."""
        response = self.session.get(url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract main content (common Docusaurus selectors)
        main_content = soup.find('div', class_='markdown') or \
                      soup.find('article') or \
                      soup.find('main')

        if not main_content:
            raise ValueError(f"Could not find main content in {url}")

        # Clean up the content (remove navigation, etc.)
        for nav in soup.find_all(['nav', 'aside', 'header', 'footer']):
            nav.decompose()

        title = soup.find('title').text if soup.find('title') else url

        return {
            'url': url,
            'title': title,
            'content': main_content.get_text(strip=True),
            'html': str(main_content)
        }
```

### 3. Text Chunking Implementation

In `src/ingestion/chunker.py`:

```python
import re
from typing import List, Dict, Any

class TextChunker:
    def __init__(self, chunk_size_tokens: int = 512, overlap_percentage: float = 0.2):
        self.chunk_size_tokens = chunk_size_tokens
        self.overlap_size = int(chunk_size_tokens * overlap_percentage)

    def chunk_text(self, text: str) -> List[Dict[str, Any]]:
        """Chunk text into segments preserving semantic boundaries."""
        # Simple tokenization based on words
        sentences = re.split(r'[.!?]+', text)
        chunks = []
        current_chunk = ""
        chunk_start = 0
        chunk_number = 0

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            # Estimate token count (rough approximation: 1 token ~ 4 chars)
            current_tokens = len(current_chunk.split())
            sentence_tokens = len(sentence.split())

            if current_tokens + sentence_tokens > self.chunk_size_tokens and current_chunk:
                # Complete current chunk and start new one
                chunks.append({
                    'content': current_chunk.strip(),
                    'start_offset': chunk_start,
                    'end_offset': chunk_start + len(current_chunk),
                    'chunk_number': chunk_number,
                    'token_count': current_tokens
                })

                # Start overlapping chunk
                overlap_start = max(0, len(current_chunk) - self.overlap_size * 4)  # Rough char estimate
                current_chunk = current_chunk[overlap_start:] + " " + sentence
                chunk_start = overlap_start
                chunk_number += 1
            else:
                current_chunk += " " + sentence

        # Add the last chunk
        if current_chunk.strip():
            chunks.append({
                'content': current_chunk.strip(),
                'start_offset': chunk_start,
                'end_offset': chunk_start + len(current_chunk),
                'chunk_number': chunk_number,
                'token_count': len(current_chunk.split())
            })

        return chunks
```

### 4. Embedding Generation Implementation

In `src/embedding/generator.py`:

```python
import cohere
import os
from typing import List, Dict, Any

class EmbeddingGenerator:
    def __init__(self):
        api_key = os.getenv('COHERE_API_KEY')
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable not set")

        self.client = cohere.Client(api_key)
        self.model = "embed-english-v3.0"

    def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Generate embeddings for text chunks."""
        texts = [chunk['content'] for chunk in chunks]

        # Batch process to respect API limits
        batch_size = 96  # Cohere's max batch size is 96
        embeddings = []

        for i in range(0, len(texts), batch_size):
            batch_texts = texts[i:i + batch_size]
            response = self.client.embed(
                texts=batch_texts,
                model=self.model,
                input_type="search_document"
            )

            for j, embedding in enumerate(response.embeddings):
                chunk_idx = i + j
                embeddings.append({
                    'chunk_id': f"chunk_{chunk_idx}",
                    'document_id': chunks[chunk_idx].get('document_id', 'unknown'),
                    'content': chunks[chunk_idx]['content'],
                    'vector': embedding,
                    'chunk_number': chunks[chunk_idx]['chunk_number'],
                    'source_url': chunks[chunk_idx].get('source_url', ''),
                    'model_used': self.model
                })

        return embeddings
```

### 5. Qdrant Storage Implementation

In `src/storage/qdrant_client.py`:

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models
import os
from typing import List, Dict, Any

class QdrantStorage:
    def __init__(self):
        url = os.getenv('QDRANT_URL')
        api_key = os.getenv('QDRANT_API_KEY')
        collection_name = os.getenv('QDRANT_COLLECTION_NAME', 'docs_embeddings')

        if not url or not api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """Create Qdrant collection if it doesn't exist."""
        try:
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
            )

    def store_embeddings(self, embeddings: List[Dict[str, Any]]) -> bool:
        """Store embeddings in Qdrant."""
        points = []
        for i, emb in enumerate(embeddings):
            points.append(
                models.PointStruct(
                    id=i,
                    vector=emb['vector'],
                    payload={
                        'source_url': emb['source_url'],
                        'document_title': emb.get('document_title', ''),
                        'chunk_content': emb['content'],
                        'chunk_id': emb['chunk_id'],
                        'document_id': emb['document_id'],
                        'chunk_number': emb['chunk_number'],
                        'model_used': emb['model_used'],
                        'created_at': emb.get('created_at', ''),
                    }
                )
            )

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return True
```

## Running the Pipeline

### 1. Set up Environment Variables

Copy `.env.example` to `.env` and add your actual API keys:

```bash
cp .env.example .env
# Edit .env with your actual keys
```

### 2. Run the Main Pipeline

```bash
cd backend
python main.py
```

## Testing

Create basic tests in the `tests/` directory:

In `tests/test_ingestion.py`:

```python
import pytest
from src.ingestion.url_fetcher import URLFetcher

def test_url_fetcher_initialization():
    fetcher = URLFetcher()
    assert fetcher.session is not None

# Add more tests as needed
```

Run tests:

```bash
pytest tests/
```

## Next Steps

1. Implement error handling and retry logic
2. Add logging for monitoring and debugging
3. Implement rate limiting to respect API limits
4. Add more sophisticated content extraction for different Docusaurus themes
5. Implement batch processing for better performance
6. Add monitoring and metrics collection