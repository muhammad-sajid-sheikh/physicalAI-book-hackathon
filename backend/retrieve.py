#!/usr/bin/env python3
"""
Qdrant Retrieval System - Retrieve relevant content from stored vectors.

This script implements a vector-based retrieval system that connects to Qdrant
and performs semantic search on existing stored vectors.
"""

import os
import logging
import time
from typing import List, Dict, Any
from dotenv import load_dotenv
from src.storage.qdrant_client import QdrantStorage
import cohere


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def retry_operation(operation, max_retries=3, delay=1, backoff=2, exceptions=(Exception,)):
    """
    Retry decorator for transient failure handling.

    Args:
        operation: Function to retry
        max_retries: Maximum number of retry attempts
        delay: Initial delay between retries (seconds)
        backoff: Multiplier for delay after each retry
        exceptions: Tuple of exceptions to catch and retry

    Returns:
        Result of the operation if successful
    """
    current_delay = delay
    for attempt in range(max_retries + 1):
        try:
            return operation()
        except exceptions as e:
            if attempt == max_retries:
                logger.error(f"Operation failed after {max_retries} retries: {str(e)}")
                raise e
            else:
                logger.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {current_delay}s...")
                time.sleep(current_delay)
                current_delay *= backoff

    raise Exception("Retry operation failed")


def embed_query(query: str) -> List[float]:
    """
    Convert text query to embedding vector using Cohere.

    Args:
        query: Text query to embed

    Returns:
        Embedding vector as list of floats
    """
    def _embed_operation():
        # Get Cohere API key from environment
        cohere_api_key = os.getenv('COHERE_API_KEY')
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable not set")

        # Initialize Cohere client
        co = cohere.Client(cohere_api_key)

        # Generate embedding for the query
        response = co.embed(
            texts=[query],
            model="embed-english-v3.0",  # Same model used for stored vectors
            input_type="search_query"
        )

        # Extract the embedding (first result since we sent one query)
        embedding = response.embeddings[0]
        logger.info(f"Successfully embedded query of length {len(query)}")
        return embedding

    try:
        result = retry_operation(_embed_operation, max_retries=3, delay=1, backoff=2,
                                exceptions=(Exception,))
        return result
    except Exception as e:
        logger.error(f"Error embedding query after retries: {str(e)}")
        raise


def validate_qdrant_connection():
    """
    Validate Qdrant connection to existing vector collection.

    Returns:
        bool: True if connection is successful, False otherwise
    """
    try:
        logger.info("Validating Qdrant connection...")

        # Create Qdrant storage instance which will automatically connect
        storage = QdrantStorage()

        # Get collection info to verify connection
        collection_info = storage.get_collection_info()

        logger.info(f"Successfully connected to Qdrant collection: {collection_info['collection_name']}")
        logger.info(f"Collection has {collection_info['point_count']} vectors")
        logger.info(f"Vector size: {collection_info['vector_size']}")
        logger.info(f"Status: {collection_info['status']}")

        return True

    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {str(e)}")
        return False


class QdrantRetriever:
    """
    Basic retrieval framework for Qdrant vector database.
    """

    def __init__(self):
        """
        Initialize the retrieval framework.
        """
        try:
            # Initialize Cohere client for query embedding
            cohere_api_key = os.getenv('COHERE_API_KEY')
            if not cohere_api_key:
                raise ValueError("COHERE_API_KEY environment variable not set")

            self.cohere_client = cohere.Client(cohere_api_key)
            self.qdrant_storage = QdrantStorage()

            logger.info("QdrantRetriever initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing QdrantRetriever: {str(e)}")
            raise

    def retrieve_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve top-k relevant text chunks based on query.

        Args:
            query: Text query to search for
            top_k: Number of results to return (default 5)

        Returns:
            List of dictionaries containing retrieved content and metadata
        """
        import time

        start_time = time.time()

        # Validate input parameters
        if not query or not query.strip():
            raise ValueError("Query cannot be empty or contain only whitespace")

        if top_k <= 0:
            raise ValueError("top_k must be a positive integer")

        if top_k > 100:  # Reasonable upper limit
            raise ValueError("top_k cannot exceed 100")

        try:
            logger.info(f"Retrieving content for query: '{query}' (top_k={top_k})")

            # Convert query to embedding
            embed_start_time = time.time()
            query_embedding = embed_query(query)
            embed_duration = time.time() - embed_start_time
            logger.debug(f"Query embedding took {embed_duration:.3f}s")

            # Perform search in Qdrant with retry logic
            search_start_time = time.time()

            def _search_operation():
                return self.qdrant_storage.search_similar(
                    query_vector=query_embedding,
                    limit=top_k
                )

            search_results = retry_operation(_search_operation, max_retries=3, delay=1, backoff=2,
                                            exceptions=(Exception,))
            search_duration = time.time() - search_start_time
            logger.debug(f"Qdrant search took {search_duration:.3f}s")

            # Format results
            format_start_time = time.time()
            formatted_results = []
            for i, result in enumerate(search_results):
                payload = result['payload']

                formatted_result = {
                    'rank': i + 1,
                    'similarity_score': result['score'],
                    'content': payload.get('chunk_content', ''),
                    'source_url': payload.get('source_url', ''),
                    'document_title': payload.get('document_title', ''),
                    'chunk_id': payload.get('chunk_id', ''),
                    'document_id': payload.get('document_id', ''),
                    'chunk_number': payload.get('chunk_number', 0),
                    'token_count': payload.get('token_count', 0),
                    'model_used': payload.get('model_used', ''),
                    'metadata': payload.get('metadata', {})
                }

                formatted_results.append(formatted_result)
            format_duration = time.time() - format_start_time
            logger.debug(f"Results formatting took {format_duration:.3f}s")

            # Log if no results found
            if len(formatted_results) == 0:
                logger.warning(f"No relevant results found for query: '{query}'")

            total_duration = time.time() - start_time
            logger.info(f"Retrieved {len(formatted_results)} results for query in {total_duration:.3f}s")

            # Performance monitoring
            perf_metrics = {
                'total_duration': total_duration,
                'embed_duration': embed_duration,
                'search_duration': search_duration,
                'format_duration': format_duration,
                'num_results': len(formatted_results),
                'query_length': len(query),
                'top_k_requested': top_k
            }

            # Log performance metrics if threshold exceeded
            if total_duration > 2.0:  # More than 2 seconds
                logger.warning(f"Slow query detected: {total_duration:.3f}s for query '{query[:50]}...'")

            return formatted_results

        except Exception as e:
            total_duration = time.time() - start_time
            logger.error(f"Error retrieving content after {total_duration:.3f}s: {str(e)}")
            raise

    def validate_results(self, query: str, results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate retrieved results to ensure content matches source URLs and metadata.

        Args:
            query: Original query that generated the results
            results: List of retrieved results to validate

        Returns:
            Validation report with details about validation status
        """
        try:
            logger.info(f"Validating {len(results)} results for query: '{query}'")

            validation_report = {
                'query': query,
                'total_results': len(results),
                'results_validated': 0,
                'metadata_validated': 0,
                'content_validated': 0,
                'validation_details': []
            }

            for result in results:
                is_valid = True
                validation_detail = {
                    'rank': result.get('rank'),
                    'valid': True,
                    'issues': []
                }

                # Check if essential metadata exists
                if not result.get('source_url'):
                    validation_detail['valid'] = False
                    validation_detail['issues'].append('Missing source_url')
                    is_valid = False

                if not result.get('document_title'):
                    validation_detail['valid'] = False
                    validation_detail['issues'].append('Missing document_title')
                    is_valid = False

                if not result.get('content'):
                    validation_detail['valid'] = False
                    validation_detail['issues'].append('Missing content')
                    is_valid = False

                # Check if content has reasonable length
                if result.get('content') and len(result['content']) < 10:
                    validation_detail['issues'].append('Content too short (< 10 characters)')
                    # Don't mark as invalid, just add as issue

                validation_report['validation_details'].append(validation_detail)

                if is_valid:
                    validation_report['results_validated'] += 1
                    # Count separately for metadata and content validation
                    if result.get('source_url') and result.get('document_title'):
                        validation_report['metadata_validated'] += 1
                    if result.get('content'):
                        validation_report['content_validated'] += 1

            logger.info(f"Validation completed for query: {query}")
            return validation_report

        except Exception as e:
            logger.error(f"Error validating results: {str(e)}")
            raise


def main():
    """
    Main function to demonstrate the retrieval system.

    Usage examples:
    - Basic retrieval: python retrieve.py
    - With specific query: Modify the test_query variable below
    """
    print("="*60)
    print("QDRANT RETRIEVAL SYSTEM - USAGE EXAMPLES")
    print("="*60)
    print("Environment configuration:")
    print(f"  COHERE_API_KEY configured: {'Yes' if os.getenv('COHERE_API_KEY') else 'No'}")
    print(f"  QDRANT_URL configured: {'Yes' if os.getenv('QDRANT_URL') else 'No'}")
    print(f"  QDRANT_API_KEY configured: {'Yes' if os.getenv('QDRANT_API_KEY') else 'No'}")
    print(f"  QDRANT_COLLECTION_NAME: {os.getenv('QDRANT_COLLECTION_NAME', 'Not set')}")
    print()

    # Validate Qdrant connection
    if validate_qdrant_connection():
        print("\n[OK] Qdrant connection validated successfully!")

        # Initialize the retriever
        try:
            retriever = QdrantRetriever()
            print("[OK] QdrantRetriever initialized successfully!")

            # Demonstrate basic retrieval functionality with various example queries
            example_queries = [
                "What is cognitive planning?",
                "Explain module 1 content",
                "How to create a document?",
                "Tutorial basics information",
                "Voice to action concepts"
            ]

            print(f"\nDemonstrating retrieval with {len(example_queries)} example queries:")
            print("-" * 60)

            for i, test_query in enumerate(example_queries, 1):
                print(f"\n{i}. Query: '{test_query}'")

                try:
                    results = retriever.retrieve_content(test_query, top_k=2)
                    print(f"   Retrieved {len(results)} results")

                    if results:
                        print("   Sample results:")
                        for result in results[:2]:  # Show first 2 results
                            print(f"     Rank {result['rank']}: Score {result['similarity_score']:.3f}")
                            print(f"       Content: {result['content'][:80]}...")
                            print(f"       Source: {result['source_url']}")
                            print()

                        # Validate the results
                        validation_report = retriever.validate_results(test_query, results)
                        valid_count = validation_report['results_validated']
                        total_count = validation_report['total_results']
                        print(f"   Validation: {valid_count}/{total_count} results valid")
                    else:
                        print("   No results found for this query.")

                except Exception as e:
                    print(f"   Error processing query: {str(e)}")

                print("-" * 40)

        except Exception as e:
            logger.error(f"Error during retrieval demonstration: {str(e)}")
            print(f"[ERROR] Error during retrieval demonstration: {str(e)}")
    else:
        print("\n[ERROR] Failed to validate Qdrant connection!")

    print("\n" + "="*60)
    print("RETRIEVAL SYSTEM DEMONSTRATION COMPLETE")
    print("="*60)


def run_interactive_demo():
    """
    Run an interactive demo allowing users to enter their own queries.
    """
    print("\nINTERACTIVE MODE")
    print("Enter queries to search the documentation (type 'quit' to exit):")

    if not validate_qdrant_connection():
        print("Cannot run interactive demo - Qdrant connection failed")
        return

    try:
        retriever = QdrantRetriever()
        print("Retriever ready! Enter your queries below:")

        while True:
            query = input("\nEnter query (or 'quit' to exit): ").strip()

            if query.lower() in ['quit', 'exit', 'q']:
                print("Exiting interactive mode...")
                break

            if not query:
                print("Please enter a valid query.")
                continue

            try:
                results = retriever.retrieve_content(query, top_k=3)

                if results:
                    print(f"\nFound {len(results)} results:")
                    for result in results:
                        print(f"  Rank {result['rank']}: Score {result['similarity_score']:.3f}")
                        print(f"    Content: {result['content'][:120]}...")
                        print(f"    Source: {result['source_url']}")
                        print()
                else:
                    print("No results found for your query.")

            except Exception as e:
                print(f"Error retrieving results: {str(e)}")

    except Exception as e:
        print(f"Error initializing retriever: {str(e)}")


if __name__ == "__main__":
    import sys

    # Check if running in interactive mode
    if len(sys.argv) > 1 and sys.argv[1] == "--interactive":
        run_interactive_demo()
    else:
        main()