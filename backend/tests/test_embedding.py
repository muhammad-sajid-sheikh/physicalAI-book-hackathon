"""Unit tests for the embedding generation components of the Docusaurus Content Ingestion Pipeline."""

import unittest
from unittest.mock import Mock, patch, MagicMock
from src.embedding.client import CohereClient
from src.embedding.generator import EmbeddingGenerator
from src.ingestion.models import TextChunk
from src.embedding.models import EmbeddingVector


class TestCohereClient(unittest.TestCase):
    """Test cases for the CohereClient class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the cohere.Client to avoid actual API calls
        with patch('src.embedding.client.cohere.Client'):
            self.client = CohereClient(api_key='test-key')

    @patch('src.embedding.client.cohere.Client')
    def test_initialization(self, mock_cohere_client):
        """Test that CohereClient initializes correctly."""
        client = CohereClient(api_key='test-key')

        # Verify that the cohere client was created with the API key
        mock_cohere_client.assert_called_once_with('test-key')
        self.assertEqual(client.model, "embed-english-v3.0")

    @patch('src.embedding.client.cohere.Client')
    def test_initialization_missing_api_key(self, mock_cohere_client):
        """Test that initialization fails when API key is missing."""
        mock_cohere_client.side_effect = Exception("API key required")

        with self.assertRaises(Exception):
            CohereClient(api_key=None)

    @patch('src.embedding.client.cohere.Client')
    def test_generate_embeddings_single_batch(self, mock_cohere_client):
        """Test generating embeddings for a small batch of texts."""
        # Mock the embed response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]  # Simplified embeddings
        mock_cohere_client_instance = Mock()
        mock_cohere_client_instance.embed.return_value = mock_response
        mock_cohere_client.return_value = mock_cohere_client_instance

        client = CohereClient(api_key='test-key')
        texts = ["Hello world", "Test embedding"]
        embeddings = client.generate_embeddings(texts)

        # Verify the response
        self.assertEqual(len(embeddings), 2)
        self.assertEqual(len(embeddings[0]), 3)  # 3 dimensions in mock
        self.assertEqual(len(embeddings[1]), 3)

        # Verify the embed method was called correctly
        mock_cohere_client_instance.embed.assert_called_once()
        call_args = mock_cohere_client_instance.embed.call_args
        self.assertEqual(call_args[1]['model'], "embed-english-v3.0")
        self.assertEqual(call_args[1]['input_type'], "search_document")

    @patch('src.embedding.client.cohere.Client')
    def test_generate_embeddings_multiple_batches(self, mock_cohere_client):
        """Test generating embeddings for a large batch that requires multiple requests."""
        # Mock the embed response for multiple calls
        mock_response1 = Mock()
        mock_response1.embeddings = [[0.1, 0.2]] * 96  # First batch
        mock_response2 = Mock()
        mock_response2.embeddings = [[0.3, 0.4]] * 10  # Second batch (total 106 texts)

        mock_cohere_client_instance = Mock()
        mock_cohere_client_instance.embed.side_effect = [mock_response1, mock_response2]
        mock_cohere_client.return_value = mock_cohere_client_instance

        client = CohereClient(api_key='test-key')
        texts = ["test"] * 106  # More than the 96 batch size
        embeddings = client.generate_embeddings(texts)

        # Verify the response
        self.assertEqual(len(embeddings), 106)
        self.assertEqual(mock_cohere_client_instance.embed.call_count, 2)  # Called twice for batches

    @patch('src.embedding.client.cohere.Client')
    def test_generate_single_embedding(self, mock_cohere_client):
        """Test generating embedding for a single text."""
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        mock_cohere_client_instance = Mock()
        mock_cohere_client_instance.embed.return_value = mock_response
        mock_cohere_client.return_value = mock_cohere_client_instance

        client = CohereClient(api_key='test-key')
        embedding = client.generate_embedding("Hello world")

        # Verify the response
        self.assertEqual(len(embedding), 3)
        self.assertEqual(embedding, [0.1, 0.2, 0.3])

    @patch('src.embedding.client.cohere.Client')
    def test_get_model_info(self, mock_cohere_client):
        """Test getting model information."""
        client = CohereClient(api_key='test-key')
        model_info = client.get_model_info()

        # Verify the model info structure
        self.assertIn('model_name', model_info)
        self.assertIn('dimensions', model_info)
        self.assertIn('input_type_options', model_info)
        self.assertEqual(model_info['model_name'], "embed-english-v3.0")
        self.assertEqual(model_info['dimensions'], 1024)  # Cohere v3 English model has 1024 dimensions


class TestEmbeddingGenerator(unittest.TestCase):
    """Test cases for the EmbeddingGenerator class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        with patch('src.embedding.generator.CohereClient'):
            self.generator = EmbeddingGenerator(api_key='test-key')

    @patch('src.embedding.generator.CohereClient')
    def test_initialization(self, mock_cohere_client):
        """Test that EmbeddingGenerator initializes correctly."""
        generator = EmbeddingGenerator(api_key='test-key')

        # Verify that CohereClient was instantiated
        mock_cohere_client.assert_called_once_with('test-key')

    def test_generate_embeddings_empty_chunks(self):
        """Test generating embeddings for empty chunk list."""
        with patch.object(self.generator.client, 'generate_embeddings') as mock_method:
            result = self.generator.generate_embeddings([])

            # Should return empty list without calling the API
            self.assertEqual(result, [])
            mock_method.assert_not_called()

    def test_generate_embeddings_single_chunk(self):
        """Test generating embedding for a single chunk."""
        # Create a test chunk
        chunk = TextChunk(
            chunk_id='chunk_1',
            document_id='doc_1',
            content='This is a test chunk.',
            start_offset=0,
            end_offset=20,
            chunk_number=0,
            token_count=6
        )

        with patch.object(self.generator.client, 'generate_embeddings') as mock_method:
            # Mock the API response
            mock_method.return_value = [[0.1, 0.2, 0.3]]

            result = self.generator.generate_embeddings([chunk])

            # Verify the result structure
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0]['chunk_id'], 'chunk_1')
            self.assertEqual(result[0]['content'], 'This is a test chunk.')
            self.assertEqual(result[0]['vector'], [0.1, 0.2, 0.3])
            self.assertEqual(result[0]['token_count'], 6)

            # Verify that the API was called with the right content
            mock_method.assert_called_once_with(['This is a test chunk.'], input_type="search_document")

    def test_generate_embeddings_multiple_chunks(self):
        """Test generating embeddings for multiple chunks."""
        # Create test chunks
        chunks = [
            TextChunk(
                chunk_id='chunk_1',
                document_id='doc_1',
                content='First chunk content.',
                start_offset=0,
                end_offset=18,
                chunk_number=0,
                token_count=4
            ),
            TextChunk(
                chunk_id='chunk_2',
                document_id='doc_1',
                content='Second chunk content.',
                start_offset=19,
                end_offset=39,
                chunk_number=1,
                token_count=4
            )
        ]

        with patch.object(self.generator.client, 'generate_embeddings') as mock_method:
            # Mock the API response with multiple embeddings
            mock_method.return_value = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

            result = self.generator.generate_embeddings(chunks)

            # Verify the result structure
            self.assertEqual(len(result), 2)
            self.assertEqual(result[0]['chunk_id'], 'chunk_1')
            self.assertEqual(result[1]['chunk_id'], 'chunk_2')
            self.assertEqual(result[0]['vector'], [0.1, 0.2, 0.3])
            self.assertEqual(result[1]['vector'], [0.4, 0.5, 0.6])

    def test_generate_single_chunk_embedding(self):
        """Test generating embedding for a single chunk using the dedicated method."""
        chunk = TextChunk(
            chunk_id='chunk_1',
            document_id='doc_1',
            content='Single chunk test.',
            start_offset=0,
            end_offset=17,
            chunk_number=0,
            token_count=4
        )

        with patch.object(self.generator, 'client') as mock_client:
            # Mock the single embedding generation
            mock_client.generate_embedding.return_value = [0.7, 0.8, 0.9]
            mock_client.get_model_info.return_value = {'model_name': 'embed-english-v3.0'}

            result = self.generator.generate_embedding_for_single_chunk(chunk)

            # Verify the result
            self.assertEqual(result['chunk_id'], 'chunk_1')
            self.assertEqual(result['content'], 'Single chunk test.')
            self.assertEqual(result['vector'], [0.7, 0.8, 0.9])
            self.assertEqual(result['token_count'], 4)

    def test_get_embedding_dimensions(self):
        """Test getting embedding dimensions."""
        with patch.object(self.generator.client, 'get_model_info') as mock_method:
            mock_method.return_value = {'dimensions': 1024}

            dimensions = self.generator.get_embedding_dimensions()

            self.assertEqual(dimensions, 1024)
            mock_method.assert_called_once()

    def test_validate_embedding_quality(self):
        """Test embedding quality validation."""
        with patch.object(self.generator, 'get_embedding_dimensions') as mock_method:
            mock_method.return_value = 3  # Using small number for test

            # Valid embedding
            valid_embedding = [0.1, 0.2, 0.3]
            self.assertTrue(self.generator.validate_embedding_quality(valid_embedding))

            # Invalid embedding - wrong dimensions
            wrong_dim_embedding = [0.1, 0.2]  # Only 2 dimensions, expected 3
            self.assertFalse(self.generator.validate_embedding_quality(wrong_dim_embedding))

            # Invalid embedding - contains NaN
            nan_embedding = [0.1, float('nan'), 0.3]
            self.assertFalse(self.generator.validate_embedding_quality(nan_embedding))

            # Invalid embedding - contains infinity
            inf_embedding = [0.1, float('inf'), 0.3]
            self.assertFalse(self.generator.validate_embedding_quality(inf_embedding))

            # Empty embedding
            self.assertFalse(self.generator.validate_embedding_quality([]))


class TestEmbeddingVector(unittest.TestCase):
    """Test cases for the EmbeddingVector class."""

    def test_embedding_vector_creation(self):
        """Test creating an EmbeddingVector instance."""
        vector = EmbeddingVector(
            embedding_id='emb_1',
            chunk_id='chunk_1',
            vector=[0.1, 0.2, 0.3],
            model_used='test-model',
            metadata={'source': 'test'}
        )

        self.assertEqual(vector.embedding_id, 'emb_1')
        self.assertEqual(vector.chunk_id, 'chunk_1')
        self.assertEqual(vector.vector, [0.1, 0.2, 0.3])
        self.assertEqual(vector.model_used, 'test-model')
        self.assertEqual(vector.metadata['source'], 'test')

    def test_embedding_vector_validation(self):
        """Test EmbeddingVector validation."""
        vector = EmbeddingVector(
            embedding_id='emb_1',
            chunk_id='chunk_1',
            vector=[0.1, 0.2, 0.3] * 341 + [0.4],  # 1024-dimensional vector
            model_used='test-model'
        )

        # Validation should pass for valid vector
        self.assertTrue(vector.validate())

    def test_embedding_vector_validation_invalid_dimensions(self):
        """Test EmbeddingVector validation with wrong dimensions."""
        vector = EmbeddingVector(
            embedding_id='emb_1',
            chunk_id='chunk_1',
            vector=[0.1, 0.2, 0.3],  # Only 3 dimensions, need 1024 for Cohere v3
            model_used='test-model'
        )

        # Validation should fail for wrong dimensions
        with self.assertRaises(ValueError):
            vector.validate()

    def test_embedding_vector_validation_with_nan(self):
        """Test EmbeddingVector validation with NaN values."""
        vector = EmbeddingVector(
            embedding_id='emb_1',
            chunk_id='chunk_1',
            vector=[0.1, float('nan'), 0.3] * 341 + [0.4],  # Contains NaN
            model_used='test-model'
        )

        # Validation should fail for NaN values
        with self.assertRaises(ValueError):
            vector.validate()

    def test_embedding_vector_validation_missing_fields(self):
        """Test EmbeddingVector validation with missing required fields."""
        # Test missing embedding_id
        with self.assertRaises(ValueError):
            EmbeddingVector(
                embedding_id='',  # Empty
                chunk_id='chunk_1',
                vector=[0.1] * 1024,
                model_used='test-model'
            )

        # Test missing chunk_id
        with self.assertRaises(ValueError):
            EmbeddingVector(
                embedding_id='emb_1',
                chunk_id='',  # Empty
                vector=[0.1] * 1024,
                model_used='test-model'
            )

        # Test missing vector
        with self.assertRaises(ValueError):
            EmbeddingVector(
                embedding_id='emb_1',
                chunk_id='chunk_1',
                vector=[],  # Empty
                model_used='test-model'
            )

        # Test missing model_used
        with self.assertRaises(ValueError):
            EmbeddingVector(
                embedding_id='emb_1',
                chunk_id='chunk_1',
                vector=[0.1] * 1024,
                model_used=''  # Empty
            )

    def test_from_generator_result(self):
        """Test creating EmbeddingVector from generator result."""
        result = {
            'chunk_id': 'chunk_1',
            'vector': [0.1, 0.2, 0.3],
            'embedding_id': 'emb_1',
            'document_id': 'doc_1',
            'source_url': 'https://example.com',
            'chunk_number': 0,
            'token_count': 10,
            'model_used': 'test-model'
        }

        vector = EmbeddingVector.from_generator_result(result)

        self.assertEqual(vector.chunk_id, 'chunk_1')
        self.assertEqual(vector.vector, [0.1, 0.2, 0.3])
        self.assertEqual(vector.embedding_id, 'emb_1')
        self.assertEqual(vector.model_used, 'test-model')
        self.assertIn('document_id', vector.metadata)
        self.assertIn('source_url', vector.metadata)
        self.assertIn('chunk_number', vector.metadata)


class TestEmbeddingIntegration(unittest.TestCase):
    """Integration tests for embedding components."""

    def test_text_chunk_to_embedding_flow(self):
        """Test the full flow from TextChunk to EmbeddingVector."""
        # Create a text chunk
        chunk = TextChunk(
            chunk_id='test_chunk',
            document_id='test_doc',
            content='This is a test sentence for embedding.',
            start_offset=0,
            end_offset=38,
            chunk_number=0,
            token_count=9
        )

        # Simulate what would happen in the generator
        # In a real scenario, this would call the Cohere API
        mock_embedding = [0.1, 0.5] * 512  # 1024-dimensional mock embedding

        # Create the embedding result dict as the generator would
        embedding_result = {
            'chunk_id': chunk.chunk_id,
            'document_id': chunk.document_id,
            'content': chunk.content,
            'vector': mock_embedding,
            'chunk_number': chunk.chunk_number,
            'source_url': 'https://example.com',  # This would come from the document
            'model_used': 'embed-english-v3.0',
            'token_count': chunk.token_count
        }

        # Create EmbeddingVector from the result
        embedding_vector = EmbeddingVector.from_generator_result(embedding_result)

        # Verify the transformation worked correctly
        self.assertEqual(embedding_vector.chunk_id, chunk.chunk_id)
        self.assertEqual(embedding_vector.vector, mock_embedding)
        self.assertEqual(embedding_vector.model_used, 'embed-english-v3.0')
        self.assertEqual(len(embedding_vector.vector), 1024)  # Cohere v3 dimensions

        # Validate the final vector
        self.assertTrue(embedding_vector.validate())


if __name__ == '__main__':
    unittest.main()