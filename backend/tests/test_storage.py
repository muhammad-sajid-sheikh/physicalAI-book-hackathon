"""Unit and integration tests for the storage components of the Docusaurus Content Ingestion Pipeline."""

import unittest
from unittest.mock import Mock, patch, MagicMock
from src.storage.qdrant_client import QdrantStorage
from src.storage.models import VectorRecord
from src.embedding.models import EmbeddingVector


class TestQdrantStorage(unittest.TestCase):
    """Test cases for the QdrantStorage class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the QdrantClient to avoid actual API calls
        with patch('src.storage.qdrant_client.QdrantClient'):
            self.storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

    @patch('src.storage.qdrant_client.QdrantClient')
    @patch('src.storage.qdrant_client.get_config')
    def test_initialization_with_config(self, mock_get_config, mock_qdrant_client):
        """Test that QdrantStorage initializes correctly with configuration."""
        # Mock config
        mock_config = Mock()
        mock_config.api.qdrant_url = 'http://config.qdrant.com'
        mock_config.api.qdrant_api_key = 'config-key'
        mock_config.api.qdrant_collection_name = 'test-collection'
        mock_get_config.return_value = mock_config

        storage = QdrantStorage()

        # Verify that QdrantClient was created with config values
        mock_qdrant_client.assert_called_once_with(
            url='http://config.qdrant.com',
            api_key='config-key'
        )
        self.assertEqual(storage.collection_name, 'test-collection')

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_initialization_missing_credentials(self, mock_qdrant_client):
        """Test that initialization fails when credentials are missing."""
        with self.assertRaises(ValueError):
            QdrantStorage(url=None, api_key=None)

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_initialization_with_parameters(self, mock_qdrant_client):
        """Test that QdrantStorage initializes correctly with explicit parameters."""
        storage = QdrantStorage(
            url='http://test.qdrant.com',
            api_key='test-key',
            collection_name='custom-collection'
        )

        self.assertEqual(storage.collection_name, 'custom-collection')
        mock_qdrant_client.assert_called_once_with(
            url='http://test.qdrant.com',
            api_key='test-key'
        )

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_store_embeddings_success(self, mock_qdrant_client):
        """Test successful storage of embeddings."""
        # Mock Qdrant client methods
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Create test embeddings
        test_embeddings = [
            {
                'vector': [0.1, 0.2, 0.3],
                'content': 'Test content 1',
                'chunk_id': 'chunk_1',
                'document_id': 'doc_1',
                'chunk_number': 0,
                'source_url': 'https://example.com/1',
                'document_title': 'Test Document 1',
                'model_used': 'test-model',
                'token_count': 10
            },
            {
                'vector': [0.4, 0.5, 0.6],
                'content': 'Test content 2',
                'chunk_id': 'chunk_2',
                'document_id': 'doc_2',
                'chunk_number': 1,
                'source_url': 'https://example.com/2',
                'document_title': 'Test Document 2',
                'model_used': 'test-model',
                'token_count': 12
            }
        ]

        # Call store_embeddings
        result = storage.store_embeddings(test_embeddings)

        # Verify the result
        self.assertTrue(result)
        # Verify that upsert was called (at least once)
        self.assertTrue(mock_client_instance.upsert.called)

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_store_embeddings_empty_list(self, mock_qdrant_client):
        """Test storing an empty list of embeddings."""
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Store empty list
        result = storage.store_embeddings([])

        # Should return True without errors
        self.assertTrue(result)
        # upsert should not be called for empty list
        mock_client_instance.upsert.assert_not_called()

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_store_embeddings_with_batch_size(self, mock_qdrant_client):
        """Test storing embeddings with custom batch size."""
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Create more embeddings than the default batch size to trigger batching
        test_embeddings = []
        for i in range(150):  # More than default batch size of 100
            test_embeddings.append({
                'vector': [0.1, 0.2, 0.3],
                'content': f'Test content {i}',
                'chunk_id': f'chunk_{i}',
                'document_id': f'doc_{i}',
                'chunk_number': i,
                'source_url': f'https://example.com/{i}',
                'document_title': f'Test Document {i}',
                'model_used': 'test-model',
                'token_count': 10
            })

        # Call store_embeddings with custom batch size
        result = storage.store_embeddings(test_embeddings, batch_size=50)

        # Verify the result
        self.assertTrue(result)
        # Should have been called 3 times: 150 items / 50 batch size = 3 batches
        self.assertEqual(mock_client_instance.upsert.call_count, 3)

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_store_embeddings_failure(self, mock_qdrant_client):
        """Test that exceptions during storage are properly handled."""
        mock_client_instance = Mock()
        mock_client_instance.upsert.side_effect = Exception("Storage failed")
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        test_embeddings = [{
            'vector': [0.1, 0.2, 0.3],
            'content': 'Test content',
            'chunk_id': 'chunk_1',
            'document_id': 'doc_1',
            'chunk_number': 0,
            'source_url': 'https://example.com',
            'document_title': 'Test Document',
            'model_used': 'test-model',
            'token_count': 10
        }]

        # Should raise the exception
        with self.assertRaises(Exception) as context:
            storage.store_embeddings(test_embeddings)

        self.assertIn("Storage failed", str(context.exception))

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_search_similar(self, mock_qdrant_client):
        """Test searching for similar vectors."""
        # Mock the search response
        mock_search_result = [
            Mock(),
            Mock()
        ]
        mock_search_result[0].id = 1
        mock_search_result[0].score = 0.95
        mock_search_result[0].payload = {'content': 'Similar content 1', 'source_url': 'https://example.com/1'}
        mock_search_result[0].vector = [0.1, 0.2, 0.3]

        mock_search_result[1].id = 2
        mock_search_result[1].score = 0.87
        mock_search_result[1].payload = {'content': 'Similar content 2', 'source_url': 'https://example.com/2'}
        mock_search_result[1].vector = [0.4, 0.5, 0.6]

        mock_client_instance = Mock()
        mock_client_instance.search.return_value = mock_search_result
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Call search_similar
        query_vector = [0.1, 0.1, 0.1]
        results = storage.search_similar(query_vector, limit=10)

        # Verify the results
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0]['id'], 1)
        self.assertEqual(results[0]['score'], 0.95)
        self.assertEqual(results[0]['payload']['content'], 'Similar content 1')

        # Verify the search was called correctly
        mock_client_instance.search.assert_called_once_with(
            collection_name=storage.collection_name,
            query_vector=query_vector,
            limit=10
        )

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_get_collection_info(self, mock_qdrant_client):
        """Test getting collection information."""
        # Mock collection info response
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 1024
        mock_collection_info.config.params.vectors.distance = 'cosine'
        mock_collection_info.points_count = 50
        mock_collection_info.status = 'green'

        mock_client_instance = Mock()
        mock_client_instance.get_collection.return_value = mock_collection_info
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Get collection info
        info = storage.get_collection_info()

        # Verify the information
        self.assertEqual(info['collection_name'], storage.collection_name)
        self.assertEqual(info['vector_size'], 1024)
        self.assertEqual(info['distance'], 'cosine')
        self.assertEqual(info['point_count'], 50)
        self.assertEqual(info['status'], 'green')

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_delete_collection(self, mock_qdrant_client):
        """Test deleting a collection."""
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Delete collection
        result = storage.delete_collection()

        # Verify the result and call
        self.assertTrue(result)
        mock_client_instance.delete_collection.assert_called_once_with(storage.collection_name)


class TestVectorRecord(unittest.TestCase):
    """Test cases for the VectorRecord class."""

    def test_vector_record_creation(self):
        """Test creating a VectorRecord instance."""
        record = VectorRecord(
            record_id='record_1',
            vector=[0.1, 0.2, 0.3],
            payload={'content': 'Test content', 'source_url': 'https://example.com'}
        )

        self.assertEqual(record.record_id, 'record_1')
        self.assertEqual(record.vector, [0.1, 0.2, 0.3])
        self.assertEqual(record.payload['content'], 'Test content')

    def test_vector_record_validation(self):
        """Test VectorRecord validation."""
        record = VectorRecord(
            record_id='record_1',
            vector=[0.1] * 1024,  # 1024-dimensional vector for Cohere v3
            payload={'content': 'Test content', 'source_url': 'https://example.com'}
        )

        # Validation should pass for valid record
        self.assertTrue(record.validate())

    def test_vector_record_validation_invalid_dimensions(self):
        """Test VectorRecord validation with wrong dimensions."""
        record = VectorRecord(
            record_id='record_1',
            vector=[0.1, 0.2, 0.3],  # Only 3 dimensions, need 1024
            payload={'content': 'Test content', 'source_url': 'https://example.com'}
        )

        # Validation should fail for wrong dimensions
        with self.assertRaises(ValueError):
            record.validate()

    def test_vector_record_validation_missing_fields(self):
        """Test VectorRecord validation with missing required fields."""
        # Test missing record_id
        with self.assertRaises(ValueError):
            VectorRecord(
                record_id='',  # Empty
                vector=[0.1] * 1024,
                payload={'content': 'Test content'}
            )

        # Test missing vector
        with self.assertRaises(ValueError):
            VectorRecord(
                record_id='record_1',
                vector=[],  # Empty
                payload={'content': 'Test content'}
            )

        # Test missing payload
        with self.assertRaises(ValueError):
            VectorRecord(
                record_id='record_1',
                vector=[0.1] * 1024,
                payload={}  # Empty
            )

    def test_vector_record_validation_with_nan(self):
        """Test VectorRecord validation with NaN values."""
        record = VectorRecord(
            record_id='record_1',
            vector=[0.1, float('nan'), 0.3] * 341 + [0.4],  # Contains NaN
            payload={'content': 'Test content', 'source_url': 'https://example.com'}
        )

        # Validation should fail for NaN values
        with self.assertRaises(ValueError):
            record.validate()

    def test_from_embedding_data(self):
        """Test creating VectorRecord from embedding data."""
        embedding_data = {
            'embedding_id': 'emb_1',
            'vector': [0.1, 0.2, 0.3],
            'source_url': 'https://example.com',
            'document_title': 'Test Document',
            'content': 'Test content',
            'chunk_id': 'chunk_1',
            'document_id': 'doc_1',
            'chunk_number': 0,
            'model_used': 'test-model',
            'token_count': 10
        }

        record = VectorRecord.from_embedding_data(embedding_data)

        self.assertEqual(record.record_id, 'emb_1')
        self.assertEqual(record.vector, [0.1, 0.2, 0.3])
        self.assertEqual(record.payload['source_url'], 'https://example.com')
        self.assertEqual(record.payload['document_title'], 'Test Document')
        self.assertEqual(record.payload['chunk_content'], 'Test content')

    def test_to_qdrant_payload(self):
        """Test converting VectorRecord to Qdrant payload format."""
        record = VectorRecord(
            record_id='record_1',
            vector=[0.1, 0.2, 0.3],
            payload={
                'source_url': 'https://example.com',
                'document_title': 'Test Document',
                'chunk_content': 'Test content',
                'chunk_id': 'chunk_1',
                'document_id': 'doc_1',
                'chunk_number': 0,
                'model_used': 'test-model',
                'token_count': 10,
                'created_at': '2023-01-01T00:00:00',
                'metadata': {'custom': 'value'}
            }
        )

        qdrant_payload = record.to_qdrant_payload()

        # Verify the payload structure
        self.assertEqual(qdrant_payload['source_url'], 'https://example.com')
        self.assertEqual(qdrant_payload['document_title'], 'Test Document')
        self.assertEqual(qdrant_payload['chunk_content'], 'Test content')
        self.assertEqual(qdrant_payload['chunk_id'], 'chunk_1')
        self.assertEqual(qdrant_payload['document_id'], 'doc_1')
        self.assertEqual(qdrant_payload['chunk_number'], 0)
        self.assertEqual(qdrant_payload['model_used'], 'test-model')
        self.assertEqual(qdrant_payload['token_count'], 10)
        self.assertEqual(qdrant_payload['metadata']['custom'], 'value')

    def test_get_searchable_content(self):
        """Test getting searchable content from VectorRecord."""
        record = VectorRecord(
            record_id='record_1',
            vector=[0.1, 0.2, 0.3],
            payload={'chunk_content': 'This is the searchable content'}
        )

        content = record.get_searchable_content()

        self.assertEqual(content, 'This is the searchable content')


class TestIntegrationStorage(unittest.TestCase):
    """Integration tests for storage components."""

    def test_embedding_to_vector_record_flow(self):
        """Test the full flow from EmbeddingVector to VectorRecord."""
        # Create an embedding vector
        embedding_vector = EmbeddingVector(
            embedding_id='emb_1',
            chunk_id='chunk_1',
            vector=[0.1, 0.5] * 512,  # 1024-dimensional vector
            model_used='embed-english-v3.0',
            metadata={
                'source_url': 'https://example.com',
                'document_title': 'Test Document',
                'chunk_number': 0,
                'token_count': 25
            }
        )

        # Convert to embedding data format (as it would be passed to storage)
        embedding_data = {
            'embedding_id': embedding_vector.embedding_id,
            'vector': embedding_vector.vector,
            'chunk_id': embedding_vector.chunk_id,
            'source_url': embedding_vector.metadata.get('source_url', ''),
            'document_title': embedding_vector.metadata.get('document_title', ''),
            'content': 'Test chunk content',  # This would come from the original chunk
            'chunk_number': embedding_vector.metadata.get('chunk_number', 0),
            'model_used': embedding_vector.model_used,
            'token_count': embedding_vector.metadata.get('token_count', 0)
        }

        # Create VectorRecord from embedding data
        vector_record = VectorRecord.from_embedding_data(embedding_data)

        # Verify the transformation
        self.assertEqual(vector_record.record_id, 'emb_1')
        self.assertEqual(vector_record.vector, embedding_vector.vector)
        self.assertEqual(len(vector_record.vector), 1024)  # Cohere v3 dimensions
        self.assertEqual(vector_record.payload['source_url'], 'https://example.com')
        self.assertEqual(vector_record.payload['document_title'], 'Test Document')

        # Validate the final record
        self.assertTrue(vector_record.validate())

    @patch('src.storage.qdrant_client.QdrantClient')
    def test_full_storage_flow(self, mock_qdrant_client):
        """Test the full storage flow from embedding data to Qdrant."""
        # Mock the Qdrant client
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        # Create storage instance
        storage = QdrantStorage(url='http://test.qdrant.com', api_key='test-key')

        # Create test embedding data (as would come from the embedding generator)
        test_embeddings = [
            {
                'vector': [0.1, 0.2, 0.3] * 341 + [0.4],  # 1024-dim vector
                'content': 'First test document content',
                'chunk_id': 'chunk_1',
                'document_id': 'doc_1',
                'chunk_number': 0,
                'source_url': 'https://example.com/1',
                'document_title': 'Test Document 1',
                'model_used': 'embed-english-v3.0',
                'token_count': 15
            },
            {
                'vector': [0.5, 0.6, 0.7] * 341 + [0.8],  # 1024-dim vector
                'content': 'Second test document content',
                'chunk_id': 'chunk_2',
                'document_id': 'doc_2',
                'chunk_number': 1,
                'source_url': 'https://example.com/2',
                'document_title': 'Test Document 2',
                'model_used': 'embed-english-v3.0',
                'token_count': 18
            }
        ]

        # Store the embeddings
        result = storage.store_embeddings(test_embeddings)

        # Verify successful storage
        self.assertTrue(result)
        # Verify that upsert was called (at least once)
        self.assertTrue(mock_client_instance.upsert.called)

        # Check that the number of points stored matches expectations
        # The upsert method should have been called with the right number of points
        # Each call to upsert should have the right structure


if __name__ == '__main__':
    unittest.main()