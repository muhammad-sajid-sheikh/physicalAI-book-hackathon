#!/usr/bin/env python3
"""
Quick test to verify the retrieval system is working and displaying results properly.
"""

import os
from dotenv import load_dotenv
from src.storage.qdrant_client import QdrantStorage
from src.embedding.generator import EmbeddingGenerator

# Load environment
load_dotenv()

def test_retrieval():
    print("Testing Qdrant Retrieval System")
    print("="*50)

    # Initialize components
    storage = QdrantStorage()
    generator = EmbeddingGenerator()

    print(f"[OK] Connected to Qdrant collection: {storage.collection_name}")

    # Get collection info
    info = storage.get_collection_info()
    print(f"[INFO] Collection stats: {info['point_count']} vectors, {info['vector_size']} dimensions")

    # Test embedding generation
    test_text = "cognitive planning"
    embedding = generator.client.generate_embedding(test_text)
    print(f"[OK] Generated embedding for '{test_text}' (length: {len(embedding)})")

    # Perform search
    results = storage.search_similar(embedding, limit=3)
    print(f"[OK] Found {len(results)} similar vectors")

    if results:
        print("\n[RESULTS] Sample Results:")
        for i, result in enumerate(results):
            print(f"  {i+1}. ID: {result['id']}")
            print(f"     Score: {result['score']:.4f}")
            print(f"     Payload keys: {list(result['payload'].keys()) if result['payload'] else 'None'}")

            # Show some payload content if available
            if result['payload']:
                content_preview = result['payload'].get('chunk_content', result['payload'].get('content', ''))[:100]
                source_url = result['payload'].get('source_url', result['payload'].get('url', 'N/A'))

                print(f"     Content: {content_preview}...")
                print(f"     Source: {source_url}")
            print()

    print("[SUCCESS] Retrieval system test completed successfully!")

if __name__ == "__main__":
    test_retrieval()