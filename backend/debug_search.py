#!/usr/bin/env python3
"""
Debug script to check the actual structure of search results from Qdrant.
"""

import os
from dotenv import load_dotenv
from src.storage.qdrant_client import QdrantStorage
from src.embedding.generator import EmbeddingGenerator

# Load environment
load_dotenv()

def debug_search():
    print("Debugging Qdrant Search Results Structure")
    print("="*60)

    # Initialize components
    storage = QdrantStorage()
    generator = EmbeddingGenerator()

    print(f"[OK] Connected to Qdrant collection: {storage.collection_name}")

    # Generate test embedding
    test_text = "test"
    embedding = generator.client.generate_embedding(test_text)
    print(f"[OK] Generated embedding (length: {len(embedding)})")

    # Perform search with debug output
    print("\nPerforming search and examining raw result structure...")

    # Try to access the client directly to see what methods are available
    available_methods = [method for method in dir(storage.client) if not method.startswith('_')]
    print(f"Available methods on client: {[m for m in available_methods if 'search' in m.lower() or 'query' in m.lower()]}")

    # Use the query_points method directly to see the raw response
    try:
        raw_results = storage.client.query_points(
            collection_name=storage.collection_name,
            query=embedding,
            limit=1,
            with_payload=True,
            with_vectors=False
        )

        print(f"\nRaw results type: {type(raw_results)}")
        print(f"Raw results length: {len(raw_results) if hasattr(raw_results, '__len__') else 'N/A'}")
        print(f"Raw results content: {raw_results}")

        if raw_results:
            print(f"\nFirst few results: {raw_results[:3] if hasattr(raw_results, '__getitem__') else raw_results}")

            if hasattr(raw_results, '__getitem__') and len(raw_results) > 0:
                first_result = raw_results[0]
                print(f"\nFirst result type: {type(first_result)}")
                print(f"First result attributes: {[attr for attr in dir(first_result) if not attr.startswith('_')]}")

                # Print all attributes of the first result
                for attr in [a for a in dir(first_result) if not a.startswith('_') and not callable(getattr(first_result, a))]:
                    try:
                        value = getattr(first_result, attr)
                        print(f"  {attr}: {value} (type: {type(value)})")
                    except:
                        print(f"  {attr}: <could not access>")
            else:
                # If results is not a list, print its attributes
                print(f"\nResult attributes: {[attr for attr in dir(raw_results) if not attr.startswith('_')]}")
                for attr in [a for a in dir(raw_results) if not a.startswith('_') and not callable(getattr(raw_results, a))]:
                    try:
                        value = getattr(raw_results, attr)
                        print(f"  {attr}: {value} (type: {type(value)})")
                    except:
                        print(f"  {attr}: <could not access>")

    except Exception as e:
        print(f"Error during direct query: {str(e)}")

if __name__ == "__main__":
    debug_search()