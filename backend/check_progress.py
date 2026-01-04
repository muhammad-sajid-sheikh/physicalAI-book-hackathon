#!/usr/bin/env python3
"""
Script to check the progress of ingestion by verifying the Qdrant collection.
"""

from src.storage.qdrant_client import QdrantStorage
from src.config import get_config
import logging
import time

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    try:
        logger.info("Connecting to Qdrant to check progress...")

        # Create Qdrant storage instance
        storage = QdrantStorage()

        # Get collection info
        collection_info = storage.get_collection_info()
        logger.info(f"Current collection info: {collection_info}")

        print(f"\nCurrent status:")
        print(f"- Collection: {collection_info['collection_name']}")
        print(f"- Points stored: {collection_info['point_count']}")
        print(f"- Vector dimensions: {collection_info['vector_size']}")
        print(f"- Distance metric: {collection_info['distance']}")
        print(f"- Status: {collection_info['status']}")

        print(f"\nNote: The ingestion script is likely still running in another process.")
        print(f"If you just ran the ingestion script, wait a few minutes and run this again")
        print(f"to see the updated count as more documents are processed.")

    except Exception as e:
        logger.error(f"Error checking progress: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()