from typing import List, Dict, Any
import logging
from .client import CohereClient
from ..ingestion.models import TextChunk
from ..config import get_config

class EmbeddingGenerator:
    """
    Generates embeddings for text chunks using the Cohere API.
    """

    def __init__(self, api_key: str = None):
        """
        Initialize the EmbeddingGenerator.

        Args:
            api_key: Cohere API key. If not provided, will try to read from configuration.
        """
        # Load configuration
        config = get_config()

        # Use provided API key or fall back to config
        cohere_api_key = api_key if api_key is not None else config.api.cohere_api_key

        self.client = CohereClient(cohere_api_key)
        self.logger = logging.getLogger(__name__)

    def generate_embeddings(self, chunks: List[TextChunk]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for a list of TextChunks.

        Args:
            chunks: List of TextChunk objects to generate embeddings for

        Returns:
            List of dictionaries containing embeddings and associated metadata
        """
        try:
            if not chunks:
                return []

            # Extract text content from chunks for embedding
            texts = [chunk.content for chunk in chunks]

            # Generate embeddings using Cohere client
            embeddings = self.client.generate_embeddings(texts, input_type="search_document")

            # Combine embeddings with chunk metadata
            result_embeddings = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                result_embeddings.append({
                    'chunk_id': chunk.chunk_id,
                    'document_id': chunk.document_id,
                    'content': chunk.content,
                    'vector': embedding,
                    'chunk_number': chunk.chunk_number,
                    'source_url': getattr(chunk, 'source_url', ''),  # May not be in TextChunk directly
                    'model_used': self.client.get_model_info()['model_name'],
                    'token_count': chunk.token_count
                })

            self.logger.info(f"Successfully generated embeddings for {len(chunks)} text chunks")
            return result_embeddings
        except Exception as e:
            self.logger.error(f"Error generating embeddings for chunks: {str(e)}")
            raise

    def generate_embedding_for_single_chunk(self, chunk: TextChunk) -> Dict[str, Any]:
        """
        Generate embedding for a single TextChunk.

        Args:
            chunk: TextChunk object to generate embedding for

        Returns:
            Dictionary containing the embedding and associated metadata
        """
        try:
            embedding = self.client.generate_embedding(chunk.content, input_type="search_document")

            result = {
                'chunk_id': chunk.chunk_id,
                'document_id': chunk.document_id,
                'content': chunk.content,
                'vector': embedding,
                'chunk_number': chunk.chunk_number,
                'model_used': self.client.get_model_info()['model_name'],
                'token_count': chunk.token_count
            }

            self.logger.info(f"Successfully generated embedding for chunk {chunk.chunk_id}")
            return result
        except Exception as e:
            self.logger.error(f"Error generating embedding for chunk {chunk.chunk_id}: {str(e)}")
            raise

    def get_embedding_dimensions(self) -> int:
        """
        Get the dimensionality of the embeddings.

        Returns:
            Number of dimensions in the embeddings
        """
        return self.client.get_model_info()['dimensions']

    def validate_embedding_quality(self, embedding: List[float]) -> bool:
        """
        Validate the quality of an embedding (basic validation).

        Args:
            embedding: The embedding vector to validate

        Returns:
            True if the embedding appears valid, False otherwise
        """
        if not embedding:
            return False

        dimensions = self.get_embedding_dimensions()
        if len(embedding) != dimensions:
            self.logger.warning(f"Embedding has {len(embedding)} dimensions, expected {dimensions}")
            return False

        # Check for NaN or infinity values
        for val in embedding:
            if not isinstance(val, (int, float)) or val != val:  # Check for NaN using val != val
                return False

        return True