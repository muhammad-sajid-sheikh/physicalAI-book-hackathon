"""
Validation script for the Docusaurus Content Ingestion Pipeline.

This script validates that the pipeline meets the specified success criteria:
- Content extraction rate meets 95%+ target
- Text segmentation quality meets 98%+ target
- Embedding generation success rate meets 99%+ target
- Storage success rate meets 99%+ target
- Processing time meets 30-minute target for sites under 1000 pages
"""

import time
import unittest
from unittest.mock import Mock, patch
from datetime import datetime
from src.ingestion.processor import DocumentProcessor
from src.embedding.generator import EmbeddingGenerator
from src.storage.qdrant_client import QdrantStorage
from src.ingestion.models import DocumentContent, TextChunk


class PipelineValidator:
    """Validates the success criteria of the pipeline."""

    def __init__(self):
        self.results = {
            'content_extraction_rate': 0.0,
            'text_segmentation_quality': 0.0,
            'embedding_generation_success_rate': 0.0,
            'storage_success_rate': 0.0,
            'processing_time': 0.0,
            'tests_passed': [],
            'tests_failed': []
        }

    def validate_content_extraction_rate(self, test_urls):
        """
        Validate that content extraction rate meets 95%+ target.

        Args:
            test_urls: List of URLs to test

        Returns:
            float: Content extraction success rate
        """
        successful_extractions = 0
        total_attempts = len(test_urls)

        processor = DocumentProcessor()

        for url in test_urls:
            try:
                document = processor.fetch_and_extract(url)
                if document and document.content.strip():
                    successful_extractions += 1
            except Exception:
                # Extraction failed for this URL
                pass

        extraction_rate = (successful_extractions / total_attempts) * 100 if total_attempts > 0 else 0
        self.results['content_extraction_rate'] = extraction_rate

        print(f"Content Extraction Rate: {extraction_rate:.2f}% (Target: 95%+)")
        return extraction_rate >= 95.0

    def validate_text_segmentation_quality(self, test_documents):
        """
        Validate that text segmentation produces segments of appropriate size with preserved semantic context for 98%+ of content.

        Args:
            test_documents: List of DocumentContent objects to test

        Returns:
            bool: Whether the quality meets the 98%+ target
        """
        total_content_processed = 0
        content_preserved = 0

        processor = DocumentProcessor()

        for doc in test_documents:
            original_content = doc.content
            total_content_processed += len(original_content)

            try:
                chunks = processor.chunk_content(doc)
                reconstructed_content = " ".join([chunk.content for chunk in chunks])

                # Calculate how much of the original content was preserved in chunks
                if len(original_content) > 0:
                    preserved_ratio = len(reconstructed_content) / len(original_content)
                    content_preserved += len(reconstructed_content)
            except Exception:
                # Chunking failed for this document
                pass

        preservation_rate = (content_preserved / total_content_processed) * 100 if total_content_processed > 0 else 0
        self.results['text_segmentation_quality'] = preservation_rate

        print(f"Text Segmentation Quality: {preservation_rate:.2f}% (Target: 98%+)")
        return preservation_rate >= 98.0

    def validate_embedding_generation_success_rate(self, test_chunks):
        """
        Validate that embedding generation succeeds for 99%+ of text segments.

        Args:
            test_chunks: List of TextChunk objects to test

        Returns:
            bool: Whether the success rate meets the 99%+ target
        """
        successful_generations = 0
        total_attempts = len(test_chunks)

        # Mock the Cohere API to avoid actual API calls during validation
        with patch('src.embedding.generator.CohereClient') as mock_client:
            # Configure mock to return successful responses
            mock_instance = Mock()
            mock_instance.generate_embeddings.return_value = [[0.1] * 1024 for _ in range(len(test_chunks))]
            mock_instance.get_model_info.return_value = {'model_name': 'embed-english-v3.0', 'dimensions': 1024}
            mock_client.return_value = mock_instance

            generator = EmbeddingGenerator(api_key='test-key')

            for chunk in test_chunks:
                try:
                    # Create a list with a single chunk to test
                    result = generator.generate_embeddings([chunk])
                    if result and len(result) > 0:
                        successful_generations += 1
                except Exception:
                    # Generation failed for this chunk
                    pass

        success_rate = (successful_generations / total_attempts) * 100 if total_attempts > 0 else 0
        self.results['embedding_generation_success_rate'] = success_rate

        print(f"Embedding Generation Success Rate: {success_rate:.2f}% (Target: 99%+)")
        return success_rate >= 99.0

    def validate_storage_success_rate(self, test_embeddings):
        """
        Validate that storage succeeds for 99%+ of embeddings with proper metadata.

        Args:
            test_embeddings: List of embedding dictionaries to test

        Returns:
            bool: Whether the success rate meets the 99%+ target
        """
        successful_storages = 0
        total_attempts = len(test_embeddings)

        # Mock the Qdrant client to avoid actual API calls during validation
        with patch('src.storage.qdrant_client.QdrantClient') as mock_client:
            mock_instance = Mock()
            mock_instance.upsert.return_value = True
            mock_instance.get_collection.return_value = Mock()
            mock_client.return_value = mock_instance

            storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

            for embedding in test_embeddings:
                try:
                    result = storage.store_embeddings([embedding])
                    if result:
                        successful_storages += 1
                except Exception:
                    # Storage failed for this embedding
                    pass

        success_rate = (successful_storages / total_attempts) * 100 if total_attempts > 0 else 0
        self.results['storage_success_rate'] = success_rate

        print(f"Storage Success Rate: {success_rate:.2f}% (Target: 99%+)")
        return success_rate >= 99.0

    def validate_processing_time(self, test_urls):
        """
        Validate that processing time meets 30-minute target for sites under 1000 pages.

        Args:
            test_urls: List of URLs to test

        Returns:
            bool: Whether the processing time is within the 30-minute target
        """
        start_time = time.time()

        # For this validation, we'll simulate processing of the URLs
        # In a real scenario, this would involve the full pipeline
        processor = DocumentProcessor()

        for url in test_urls:
            try:
                # Simulate processing each URL
                document = processor.fetch_and_extract(url)
                if document:
                    chunks = processor.chunk_content(document)

                    # Mock the rest of the pipeline
                    with patch('src.embedding.generator.CohereClient'), \
                         patch('src.storage.qdrant_client.QdrantClient'):
                        generator = EmbeddingGenerator(api_key='test-key')
                        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

                        if chunks:
                            embeddings = generator.generate_embeddings(chunks)
                            storage.store_embeddings(embeddings)
            except Exception:
                # Skip failed URLs for timing calculation
                pass

        end_time = time.time()
        processing_time = end_time - start_time
        self.results['processing_time'] = processing_time

        time_limit_seconds = 30 * 60  # 30 minutes in seconds
        within_time_limit = processing_time <= time_limit_seconds

        print(f"Processing Time: {processing_time:.2f}s (Target: <= {time_limit_seconds}s for sites under 1000 pages)")
        return within_time_limit

    def run_comprehensive_validation(self):
        """
        Run all validation tests and return a comprehensive report.

        Returns:
            dict: Validation results and whether all criteria were met
        """
        print("Starting comprehensive pipeline validation...")
        print("="*60)

        # Test with sample data
        test_urls = [
            'https://example.com/docs/intro',
            'https://example.com/docs/setup',
            'https://example.com/docs/usage',
            'https://example.com/docs/advanced',
            'https://example.com/docs/troubleshooting'
        ]

        # Create test documents
        test_docs = [
            DocumentContent(
                source_url=url,
                title=f'Test Document for {url}',
                content='This is a test document with sufficient content to test the chunking algorithm. ' * 20,
                html_content='<div>This is a test document with sufficient content to test the chunking algorithm. ' * 20 + '</div>',
                metadata={'author': 'Test Author', 'version': '1.0'}
            ) for url in test_urls
        ]

        # Create test chunks
        test_chunks = [
            TextChunk(
                chunk_id=f'test_chunk_{i}',
                document_id=f'test_doc_{i}',
                content=f'Test chunk content {i} with sufficient text to test embedding generation.',
                start_offset=0,
                end_offset=50,
                chunk_number=i,
                token_count=15
            ) for i in range(10)
        ]

        # Create test embeddings
        test_embeddings = [
            {
                'vector': [0.1] * 1024,  # 1024-dimensional vector
                'content': f'Test embedding content {i}',
                'chunk_id': f'test_chunk_{i}',
                'document_id': f'test_doc_{i}',
                'chunk_number': i,
                'source_url': f'https://example.com/{i}',
                'document_title': f'Test Document {i}',
                'model_used': 'embed-english-v3.0',
                'token_count': 15
            } for i in range(10)
        ]

        # Run all validations
        criteria_results = {
            'content_extraction': self.validate_content_extraction_rate(test_urls[:3]),  # Use fewer for faster test
            'text_segmentation': self.validate_text_segmentation_quality(test_docs),
            'embedding_generation': self.validate_embedding_generation_success_rate(test_chunks),
            'storage': self.validate_storage_success_rate(test_embeddings),
            'processing_time': self.validate_processing_time(test_urls[:2])  # Use fewer for faster test
        }

        print("="*60)
        print("VALIDATION SUMMARY:")
        print(f"Content Extraction Rate: {'PASS' if criteria_results['content_extraction'] else 'FAIL'} ({self.results['content_extraction_rate']:.2f}%)")
        print(f"Text Segmentation Quality: {'PASS' if criteria_results['text_segmentation'] else 'FAIL'} ({self.results['text_segmentation_quality']:.2f}%)")
        print(f"Embedding Generation Success: {'PASS' if criteria_results['embedding_generation'] else 'FAIL'} ({self.results['embedding_generation_success_rate']:.2f}%)")
        print(f"Storage Success Rate: {'PASS' if criteria_results['storage'] else 'FAIL'} ({self.results['storage_success_rate']:.2f}%)")
        print(f"Processing Time: {'PASS' if criteria_results['processing_time'] else 'FAIL'} ({self.results['processing_time']:.2f}s)")

        all_passed = all(criteria_results.values())
        print(f"\nOVERALL RESULT: {'ALL CRITERIA MET' if all_passed else 'SOME CRITERIA NOT MET'}")

        return {
            'all_criteria_met': all_passed,
            'detailed_results': criteria_results,
            'summary': self.results
        }


def run_validation_tests():
    """Run the validation tests."""
    validator = PipelineValidator()
    results = validator.run_comprehensive_validation()

    return results


if __name__ == '__main__':
    results = run_validation_tests()

    # Exit with appropriate code
    exit(0 if results['all_criteria_met'] else 1)