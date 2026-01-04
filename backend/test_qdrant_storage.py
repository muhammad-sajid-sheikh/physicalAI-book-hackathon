#!/usr/bin/env python3
"""
Script to test Qdrant storage functionality directly.
"""

from src.storage.qdrant_client import QdrantStorage
from src.config import get_config
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    try:
        logger.info("Testing Qdrant storage functionality...")

        # Create Qdrant storage instance
        storage = QdrantStorage()

        # Get initial collection info
        initial_info = storage.get_collection_info()
        logger.info(f"Initial collection info: {initial_info}")

        # Create a simple test embedding to store
        test_embeddings = [{
            'vector': [0.1] * 1024,  # 1024-dimensional vector
            'content': 'Test content for verification',
            'chunk_id': 'test_chunk_1',
            'document_id': 'test_doc_1',
            'chunk_number': 0,
            'source_url': 'https://test.example.com',
            'document_title': 'Test Document',
            'model_used': 'embed-english-v3.0',
            'token_count': 10
        }]

        logger.info("Attempting to store test embedding...")
        result = storage.store_embeddings(test_embeddings)

        if result:
            logger.info("Successfully stored test embedding!")

            # Check collection info after storage
            final_info = storage.get_collection_info()
            logger.info(f"Final collection info: {final_info}")

            points_added = final_info['point_count'] - initial_info['point_count']
            logger.info(f"Points added: {points_added}")
        else:
            logger.error("Failed to store test embedding")

    except Exception as e:
        logger.error(f"Error testing Qdrant storage: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()