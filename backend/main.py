#!/usr/bin/env python3
"""
Main entry point for the Docusaurus Content Ingestion Pipeline.

This script orchestrates the full pipeline from URL fetching to vector storage:
1. Fetch content from Docusaurus URLs
2. Extract and clean the text content
3. Chunk the content into semantic segments
4. Generate embeddings using Cohere models
5. Store embeddings in Qdrant vector database
"""

import argparse
import logging
import sys
import os
from dotenv import load_dotenv
from src.ingestion.processor import DocumentProcessor
from src.embedding.generator import EmbeddingGenerator
from src.storage.qdrant_client import QdrantStorage
from src.config import get_config
from src.logging_config import setup_logging, PipelineLogger

# Load environment variables
load_dotenv()

# Configure logging
logger = setup_logging(level="INFO", log_file="logs/pipeline.log")
logger = PipelineLogger(__name__)


def main():
    """Main function to run the full ingestion pipeline end-to-end."""
    parser = argparse.ArgumentParser(description='Docusaurus Content Ingestion Pipeline')
    parser.add_argument('urls', nargs='+', help='URLs to process')
    parser.add_argument('--chunk-size', type=int, help='Target size for text chunks in tokens')
    parser.add_argument('--overlap-percent', type=float, help='Overlap percentage between chunks')

    args = parser.parse_args()

    logger.info("Starting Docusaurus Content Ingestion Pipeline")

    try:
        # Load configuration
        config = get_config()
        logger.info("Configuration loaded successfully", extra={
            'chunk_size_tokens': config.processing.chunk_size_tokens,
            'overlap_percentage': config.processing.overlap_percentage,
            'batch_size': config.processing.batch_size,
            'timeout': config.processing.timeout
        })

        # Override config values with command line arguments if provided
        chunk_size = args.chunk_size if args.chunk_size is not None else config.processing.chunk_size_tokens
        overlap_percent = args.overlap_percent if args.overlap_percent is not None else config.processing.overlap_percentage

        # Initialize components
        logger.info("Initializing pipeline components...")

        # Document processor for fetching and chunking
        processor = DocumentProcessor(
            chunk_size_tokens=chunk_size,
            overlap_percentage=overlap_percent
        )

        # Embedding generator for creating vector representations
        embedder = EmbeddingGenerator()

        # Qdrant storage for storing vectors
        storage = QdrantStorage()

        logger.info("Components initialized successfully")

        # Track processing statistics
        total_urls = len(args.urls)
        processed_urls = 0
        failed_urls = 0
        total_chunks = 0
        total_embeddings = 0

        # Process each URL
        for url in args.urls:
            try:
                logger.info(f"Processing URL: {url}")

                # Step 1: Fetch and extract content
                logger.info("Fetching and extracting content...")
                document = processor.fetch_and_extract(url)
                logger.info(f"Successfully extracted content: {document.title}")

                # Step 2: Chunk the content
                logger.info("Chunking content...")
                text_chunks = processor.chunk_content(document)
                logger.info(f"Content chunked into {len(text_chunks)} segments")
                total_chunks += len(text_chunks)

                # Step 3: Generate embeddings
                logger.info("Generating embeddings...")
                chunk_dicts = []
                for chunk in text_chunks:
                    # Convert TextChunk to the format expected by the embedder
                    chunk_dict = {
                        'content': chunk.content,
                        'chunk_id': chunk.chunk_id,
                        'document_id': chunk.document_id,
                        'chunk_number': chunk.chunk_number,
                        'token_count': chunk.token_count,
                        'source_url': document.source_url,
                        'document_title': document.title
                    }
                    chunk_dicts.append(chunk_dict)

                embeddings = embedder.generate_embeddings(text_chunks)
                logger.info(f"Generated embeddings for {len(embeddings)} chunks")
                total_embeddings += len(embeddings)

                # Step 4: Store embeddings in Qdrant
                logger.info("Storing embeddings in Qdrant...")
                storage.store_embeddings(embeddings)
                logger.info("Successfully stored embeddings in Qdrant")

                logger.info(f"Completed processing: {url}")
                processed_urls += 1

            except Exception as url_error:
                logger.error(f"Failed to process URL {url}: {str(url_error)}", exc_info=True)
                failed_urls += 1
                # Continue with other URLs even if one fails

        logger.info(f"Pipeline completed. Processed: {processed_urls}/{total_urls} URLs, "
                   f"Failed: {failed_urls}, Total Chunks: {total_chunks}, Total Embeddings: {total_embeddings}")

        if failed_urls == total_urls:
            logger.critical("All URLs failed to process!")
            return 1
        elif failed_urls > 0:
            logger.warning(f"Partial success: {failed_urls} out of {total_urls} URLs failed to process")
            return 0  # Still return success if at least some URLs were processed
        else:
            logger.info("All URLs processed successfully!")
            return 0

    except KeyboardInterrupt:
        logger.critical("Pipeline interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Critical error during pipeline execution: {str(e)}", exc_info=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())