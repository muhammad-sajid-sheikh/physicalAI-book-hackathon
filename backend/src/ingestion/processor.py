from typing import List, Dict, Any
import logging
from .url_fetcher import URLFetcher
from .models import DocumentContent, TextChunk
from .chunker import TextChunker
from .text_cleaner import TextCleaner
from ..config import get_config
from ..logging_config import log_retry_attempt

class DocumentProcessor:
    """
    Orchestrates the entire document processing pipeline:
    1. Fetch content from URLs
    2. Extract and clean content
    3. Convert to DocumentContent entities
    4. Chunk content into semantic segments
    """

    def __init__(self, chunk_size_tokens: int = None, overlap_percentage: float = None):
        """
        Initialize the DocumentProcessor.

        Args:
            chunk_size_tokens: Target size for text chunks in tokens (optional, uses config if not provided)
            overlap_percentage: Percentage overlap between chunks (optional, uses config if not provided)
        """
        # Load configuration
        config = get_config()

        # Use provided values or fall back to config
        chunk_size = chunk_size_tokens if chunk_size_tokens is not None else config.processing.chunk_size_tokens
        overlap_percent = overlap_percentage if overlap_percentage is not None else config.processing.overlap_percentage

        self.fetcher = URLFetcher(timeout=config.processing.timeout)
        self.chunker = TextChunker(chunk_size_tokens=chunk_size, overlap_percentage=overlap_percent)
        self.cleaner = TextCleaner()
        self.config = config
        self.logger = logging.getLogger(__name__)

    @log_retry_attempt
    def fetch_and_extract(self, url: str) -> DocumentContent:
        """
        Fetch content from a URL and convert it to a DocumentContent entity.

        Args:
            url: The URL to fetch content from

        Returns:
            DocumentContent entity with extracted content
        """
        try:
            self.logger.info(f"Starting to fetch and extract content from {url}")

            # Fetch content using URLFetcher
            fetch_result = self.fetcher.fetch_content(url)

            # Convert to DocumentContent entity
            document = DocumentContent.from_url_fetcher_result(fetch_result)

            # Validate the document
            document.validate()

            self.logger.info(f"Successfully processed document from {url}", extra={
                'url': url,
                'title': document.title,
                'content_length': len(document.content),
                'source_url': document.source_url
            })
            return document
        except Exception as e:
            self.logger.error(f"Error processing document from {url}: {str(e)}", exc_info=True, extra={
                'url': url
            })
            raise

    def process_multiple_urls(self, urls: List[str]) -> List[DocumentContent]:
        """
        Process multiple URLs in sequence.

        Args:
            urls: List of URLs to process

        Returns:
            List of DocumentContent entities
        """
        documents = []
        for url in urls:
            try:
                document = self.fetch_and_extract(url)
                documents.append(document)
            except Exception as e:
                self.logger.error(f"Failed to process {url}: {str(e)}")
                # Continue with other URLs even if one fails
                continue

        return documents

    def chunk_content(self, document: DocumentContent) -> List[TextChunk]:
        """
        Chunk the content of a document into TextChunk entities.

        Args:
            document: DocumentContent entity to chunk

        Returns:
            List of TextChunk entities with content and metadata
        """
        try:
            self.logger.info(f"Starting to chunk content for document: {document.title}", extra={
                'document_title': document.title,
                'source_url': document.source_url,
                'content_length': len(document.content),
                'document_id': str(hash(document.source_url))
            })

            # Clean the content before chunking
            cleaned_content = self.cleaner.clean_text(document.content)

            # Create a temporary document with cleaned content
            temp_doc = DocumentContent(
                source_url=document.source_url,
                title=document.title,
                content=cleaned_content,
                html_content=document.html_content,
                metadata=document.metadata,
                created_at=document.created_at
            )

            # Chunk the cleaned content
            chunk_dicts = self.chunker.chunk_text(temp_doc.content)

            # Convert chunk dictionaries to TextChunk entities
            text_chunks = []
            for i, chunk_dict in enumerate(chunk_dicts):
                chunk_dict['document_id'] = str(hash(document.source_url))  # Simple ID based on URL
                chunk_dict['source_url'] = document.source_url
                chunk_dict['document_title'] = document.title
                chunk_dict['chunk_id'] = f"{chunk_dict['document_id']}_{i}"

                # Create TextChunk entity from chunk dict
                text_chunk = TextChunk.from_chunk_dict(chunk_dict, chunk_dict['document_id'])

                # Validate the chunk
                text_chunk.validate()

                text_chunks.append(text_chunk)

            self.logger.info(f"Successfully chunked document from {document.source_url} into {len(text_chunks)} chunks", extra={
                'document_title': document.title,
                'source_url': document.source_url,
                'total_chunks': len(text_chunks),
                'chunk_size_tokens': self.chunker.chunk_size_tokens,
                'overlap_percentage': self.chunker.overlap_percentage
            })
            return text_chunks
        except Exception as e:
            self.logger.error(f"Error chunking content from {document.source_url}: {str(e)}", exc_info=True, extra={
                'document_title': document.title,
                'source_url': document.source_url,
                'content_length': len(document.content)
            })
            raise

    def get_default_config(self) -> Dict[str, Any]:
        """
        Get default configuration for the processor.

        Returns:
            Dictionary with default configuration values
        """
        return {
            'chunk_size_tokens': 512,
            'overlap_percentage': 0.2,
            'batch_size': 10,
            'rate_limit_delay': 1.0,
            'max_retries': 3
        }