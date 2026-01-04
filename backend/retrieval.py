#!/usr/bin/env python3
"""
Qdrant Retrieval System - Retrieve relevant content from stored vectors.
"""

import os
import logging
from typing import List, Dict, Any
from dotenv import load_dotenv
from src.config import get_config
from src.storage.qdrant_client import QdrantStorage
import cohere
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class QdrantRetriever:
    """
    System for retrieving relevant content from Qdrant vector database.
    """

    def __init__(self):
        """
        Initialize the retrieval system.
        """
        self.config = get_config()
        self.qdrant_storage = QdrantStorage()

        # Initialize Cohere client for query embedding
        self.cohere_client = cohere.Client(self.config.api.cohere_api_key)

        logger.info("QdrantRetriever initialized successfully")

    def _embed_query(self, query: str) -> List[float]:
        """
        Convert text query to embedding vector using Cohere.

        Args:
            query: Text query to embed

        Returns:
            Embedding vector as list of floats
        """
        try:
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",  # Same model used for stored vectors
                input_type="search_query"
            )

            # Extract the embedding (first result since we sent one query)
            embedding = response.embeddings[0]
            logger.info(f"Successfully embedded query of length {len(query)}")
            return embedding

        except Exception as e:
            logger.error(f"Error embedding query: {str(e)}")
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
        try:
            logger.info(f"Retrieving content for query: '{query}' (top_k={top_k})")

            # Convert query to embedding
            query_embedding = self._embed_query(query)

            # Perform search in Qdrant
            search_results = self.qdrant_storage.search_similar(
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
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

            logger.info(f"Retrieved {len(formatted_results)} results for query")
            return formatted_results

        except Exception as e:
            logger.error(f"Error retrieving content: {str(e)}")
            raise

    def validate_retrieval(self, query: str, top_k: int = 3) -> Dict[str, Any]:
        """
        Validate retrieval by checking metadata integrity and content relevance.

        Args:
            query: Query to test
            top_k: Number of results to validate

        Returns:
            Validation results dictionary
        """
        try:
            results = self.retrieve_content(query, top_k)

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
                    'rank': result['rank'],
                    'valid': True,
                    'issues': []
                }

                # Check if essential metadata exists
                if not result['source_url']:
                    validation_detail['valid'] = False
                    validation_detail['issues'].append('Missing source_url')
                    is_valid = False

                if not result['document_title']:
                    validation_detail['valid'] = False
                    validation_detail['issues'].append('Missing document_title')
                    is_valid = False

                if not result['content']:
                    validation_detail['valid'] = False
                    validation_detail['issues'].append('Missing content')
                    is_valid = False

                validation_report['validation_details'].append(validation_detail)

                if is_valid:
                    validation_report['results_validated'] += 1
                    validation_report['metadata_validated'] += 1
                    validation_report['content_validated'] += 1

            logger.info(f"Validation completed for query: {query}")
            return validation_report

        except Exception as e:
            logger.error(f"Error validating retrieval: {str(e)}")
            raise


def main():
    """
    Main function to demonstrate the retrieval system.
    """
    try:
        logger.info("Initializing Qdrant Retrieval System")

        # Initialize retriever
        retriever = QdrantRetriever()

        # Test queries
        test_queries = [
            "What is the introduction about?",
            "Tell me about module 1 content",
            "Explain cognitive planning concepts",
            "What are the tutorial basics?",
            "How to create a document?"
        ]

        print("=" * 80)
        print("QDRANT RETRIEVAL SYSTEM")
        print("=" * 80)

        for query in test_queries:
            print(f"\nQuery: {query}")
            print("-" * 50)

            try:
                # Retrieve content
                results = retriever.retrieve_content(query, top_k=3)

                if results:
                    for result in results:
                        print(f"Rank {result['rank']} (Score: {result['similarity_score']:.3f})")
                        print(f"  Source: {result['source_url']}")
                        print(f"  Title: {result['document_title']}")
                        print(f"  Content Preview: {result['content'][:100]}...")
                        print(f"  Chunk ID: {result['chunk_id']}")
                        print()
                else:
                    print("  No results found for this query.")
                    print()

            except Exception as e:
                logger.error(f"Error processing query '{query}': {str(e)}")
                print(f"  Error retrieving content: {str(e)}")
                print()

        # Validate one query
        print("VALIDATION REPORT")
        print("-" * 50)
        validation = retriever.validate_retrieval("What is cognitive planning?", top_k=2)
        print(f"Query: {validation['query']}")
        print(f"Total Results: {validation['total_results']}")
        print(f"Valid Results: {validation['results_validated']}")
        print(f"Metadata Valid: {validation['metadata_validated']}")
        print(f"Content Valid: {validation['content_validated']}")

        for detail in validation['validation_details']:
            status = "✓" if detail['valid'] else "✗"
            issues = f" (Issues: {', '.join(detail['issues'])})" if detail['issues'] else ""
            print(f"  {status} Rank {detail['rank']}{issues}")

        print("\n" + "=" * 80)
        print("RETRIEVAL SYSTEM TEST COMPLETE")
        print("=" * 80)

    except Exception as e:
        logger.error(f"Error in main retrieval function: {str(e)}")
        raise


if __name__ == "__main__":
    main()