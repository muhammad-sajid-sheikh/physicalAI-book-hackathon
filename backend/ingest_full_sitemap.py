#!/usr/bin/env python3
"""
Script to ingest ALL content from the Physical AI Book website into Qdrant.
"""

import requests
from bs4 import BeautifulSoup
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
from urllib.parse import urljoin, urlparse

# Load environment variables
load_dotenv()

# Configure logging
logger = setup_logging(level="INFO", log_file="logs/pipeline.log")
logger = PipelineLogger(__name__)


def extract_urls_from_sitemap(sitemap_url: str) -> list:
    """
    Extract URLs from a sitemap XML file, filtering for the correct domain.

    Args:
        sitemap_url: URL of the sitemap.xml file

    Returns:
        List of URLs extracted from the sitemap
    """
    try:
        logger.info(f"Fetching sitemap from {sitemap_url}")
        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; DocusaurusBot/1.0)'
        }
        response = requests.get(sitemap_url, headers=headers, timeout=30)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'xml')  # Use 'xml' parser for sitemaps
        urls = []

        # Look for <url><loc>URL</loc></url> patterns in sitemap
        for url_tag in soup.find_all('url'):
            loc_tag = url_tag.find('loc')
            if loc_tag:
                url = loc_tag.text.strip()
                # Filter out non-document URLs if needed
                if url and url.startswith('http'):
                    # Only include URLs from the main site domain
                    if 'physical-ai-book-hackathon-mauve.vercel.app' in url:
                        # Skip certain URL patterns that are not content pages
                        if not any(skip in url.lower() for skip in ['.jpg', '.png', '.pdf', '.zip', 'assets/', 'static/', 'blog/']):
                            urls.append(url)

        # If no URLs found with 'url' tag, try 'sitemap' for nested sitemaps
        if not urls:
            for sitemap_tag in soup.find_all('sitemap'):
                loc_tag = sitemap_tag.find('loc')
                if loc_tag:
                    nested_sitemap_url = loc_tag.text.strip()
                    logger.info(f"Found nested sitemap: {nested_sitemap_url}")
                    # Recursively get URLs from nested sitemap
                    nested_urls = extract_urls_from_sitemap(nested_sitemap_url)
                    # Filter nested URLs to only include the correct domain
                    nested_urls = [url for url in nested_urls if 'physical-ai-book-hackathon-mauve.vercel.app' in url]
                    urls.extend(nested_urls)

        logger.info(f"Extracted {len(urls)} URLs from sitemap")
        return urls

    except Exception as e:
        logger.error(f"Error extracting URLs from sitemap: {str(e)}", exc_info=True)
        return []


def main():
    """Main function to run the full ingestion pipeline with all sitemap URLs."""
    try:
        # Load configuration
        config = get_config()
        logger.info("Configuration loaded successfully", extra={
            'chunk_size_tokens': config.processing.chunk_size_tokens,
            'overlap_percentage': config.processing.overlap_percentage,
            'batch_size': config.processing.batch_size,
            'timeout': config.processing.timeout
        })

        # Extract ALL URLs from the sitemap
        sitemap_url = "https://physical-ai-book-hackathon-mauve.vercel.app/sitemap.xml"
        all_urls = extract_urls_from_sitemap(sitemap_url)

        # Filter out blog URLs and focus on docs
        docs_urls = [url for url in all_urls if '/docs/' in url]
        logger.info(f"Found {len(docs_urls)} documentation URLs out of {len(all_urls)} total URLs")

        if not docs_urls:
            logger.warning("No documentation URLs found in sitemap.")
            return 1

        logger.info(f"Processing {len(docs_urls)} documentation URLs from the website")

        # Initialize components
        logger.info("Initializing pipeline components...")
        processor = DocumentProcessor()
        embedder = EmbeddingGenerator()
        storage = QdrantStorage()

        logger.info("Components initialized successfully")

        # Track processing statistics
        total_urls = len(docs_urls)
        processed_urls = 0
        failed_urls = 0
        total_chunks = 0
        total_embeddings = 0

        # Process each URL
        for i, url in enumerate(docs_urls):
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
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}", exc_info=True)

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