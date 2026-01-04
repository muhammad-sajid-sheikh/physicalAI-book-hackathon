#!/usr/bin/env python3
"""
Script to verify that the data was successfully ingested into Qdrant.
"""

from src.storage.qdrant_client import QdrantStorage
from src.config import get_config
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    try:
        logger.info("Connecting to Qdrant to verify ingestion...")

        # Create Qdrant storage instance
        storage = QdrantStorage()

        # Get collection info
        collection_info = storage.get_collection_info()
        logger.info(f"Collection info: {collection_info}")

        # Check if there are points in the collection
        if collection_info['point_count'] > 0:
            logger.info(f"✅ SUCCESS: Data has been successfully ingested into Qdrant!")
            logger.info(f"   - Collection: {collection_info['collection_name']}")
            logger.info(f"   - Points stored: {collection_info['point_count']}")
            logger.info(f"   - Vector dimensions: {collection_info['vector_size']}")
            logger.info(f"   - Distance metric: {collection_info['distance']}")
            logger.info(f"   - Status: {collection_info['status']}")

            # Try a sample search to verify data is queryable
            try:
                # Create a sample query vector (1024 dimensions for Cohere embeddings)
                sample_query = [0.1] * 1024
                search_results = storage.search_similar(sample_query, limit=3)
                logger.info(f"✅ Sample search successful: Found {len(search_results)} similar vectors")

                if search_results:
                    logger.info("Sample result payload keys:", list(search_results[0]['payload'].keys()))

            except Exception as e:
                logger.error(f"Error during sample search: {str(e)}")

        else:
            logger.warning("⚠️  No points found in the collection. Ingestion may not have completed successfully.")

    except Exception as e:
        logger.error(f"❌ Error verifying ingestion: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()