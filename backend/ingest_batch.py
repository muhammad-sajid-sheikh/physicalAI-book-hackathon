#!/usr/bin/env python3
"""
Script to ingest content from the Physical AI Book website in smaller batches.
"""

import logging
import sys
import os
from dotenv import load_dotenv
from src.ingestion.processor import DocumentProcessor
from src.embedding.generator import EmbeddingGenerator
from src.storage.qdrant_client import QdrantStorage
from src.config import get_config
from src.logging_config import setup_logging, PipelineLogger
import time

# Load environment variables
load_dotenv()

# Configure logging
logger = setup_logging(level="INFO", log_file="logs/pipeline.log")
logger = PipelineLogger(__name__)


def get_main_site_urls():
    """
    Get the list of documentation URLs from the main Physical AI Book site.
    """
    base_url = "https://physical-ai-book-hackathon-mauve.vercel.app"

    doc_paths = [
        "/docs/intro",
        "/docs/introduction",
        "/docs/module-04/",
        "/docs/module-04/autonomous-capstone",
        "/docs/module-04/cognitive-planning",
        "/docs/module-04/voice-to-action",
        "/docs/module-1/",
        "/docs/module-1/chapter-1-1",
        "/docs/module-1/chapter-1-2",
        "/docs/module-1/chapter-1-3",
        "/docs/module-2/",
        "/docs/module-2/chapter-2-1",
        "/docs/module-2/chapter-2-2",
        "/docs/module-2/chapter-2-3",
        "/docs/module-3/",
        "/docs/module-3/chapter-3-1",
        "/docs/module-3/chapter-3-2",
        "/docs/module-3/chapter-3-3",
        "/docs/tutorial-basics/congratulations",
        "/docs/tutorial-basics/create-a-blog-post",
        "/docs/tutorial-basics/create-a-document",
        "/docs/tutorial-basics/create-a-page",
        "/docs/tutorial-basics/deploy-your-site",
        "/docs/tutorial-basics/markdown-features",
        "/docs/tutorial-extras/manage-docs-versions",
        "/docs/tutorial-extras/translate-your-site",
    ]

    urls = [base_url + path for path in doc_paths]
    return urls


def main():
    """Main function to run the ingestion pipeline with main site URLs."""
    try:
        # Load configuration
        config = get_config()
        logger.info("Configuration loaded successfully", extra={
            'chunk_size_tokens': config.processing.chunk_size_tokens,
            'overlap_percentage': config.processing.overlap_percentage,
            'batch_size': config.processing.batch_size,
            'timeout': config.processing.timeout
        })

        # Get URLs from the main site
        all_urls = get_main_site_urls()
        logger.info(f"Found {len(all_urls)} documentation URLs from the main site")

        if not all_urls:
            logger.warning("No documentation URLs found.")
            return 1

        logger.info(f"Processing {len(all_urls)} documentation URLs from the main website")

        # Initialize components
        logger.info("Initializing pipeline components...")
        processor = DocumentProcessor()
        embedder = EmbeddingGenerator()
        storage = QdrantStorage()

        logger.info("Components initialized successfully")

        # Track processing statistics
        total_urls = len(all_urls)
        processed_urls = 0
        failed_urls = 0
        total_chunks = 0
        total_embeddings = 0

        # Process each URL with better error handling
        for i, url in enumerate(all_urls):
            try:
                logger.info(f"Processing URL {i+1}/{total_urls}: {url}")

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
                embeddings = embedder.generate_embeddings(text_chunks)
                logger.info(f"Generated embeddings for {len(embeddings)} chunks")
                total_embeddings += len(embeddings)

                # Step 4: Store embeddings in Qdrant
                logger.info("Storing embeddings in Qdrant...")
                storage.store_embeddings(embeddings)
                logger.info("Successfully stored embeddings in Qdrant")

                logger.info(f"Completed processing: {url}")
                processed_urls += 1

                # Check collection size after each URL
                try:
                    collection_info = storage.get_collection_info()
                    logger.info(f"Current collection size: {collection_info['point_count']} points")
                except Exception as e:
                    logger.warning(f"Could not get collection info: {str(e)}")

                # Add a small delay to be respectful to the server
                time.sleep(1)

            except Exception as url_error:
                logger.error(f"Failed to process URL {url}: {str(url_error)}", exc_info=True)
                failed_urls += 1
                # Continue with other URLs even if one fails

        logger.info(f"Pipeline completed. Processed: {processed_urls}/{total_urls} URLs, "
                   f"Failed: {failed_urls}, Total Chunks: {total_chunks}, Total Embeddings: {total_embeddings}")

        # Get collection info to verify all data was stored
        try:
            collection_info = storage.get_collection_info()
            logger.info(f"Final collection info: {collection_info}")

            # Show how many more documents were added
            added_points = collection_info['point_count'] - 4  # Assuming we started with 4
            logger.info(f"Added {added_points} new points to the collection")
        except Exception as e:
            logger.error(f"Error getting final collection info: {str(e)}", exc_info=True)

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