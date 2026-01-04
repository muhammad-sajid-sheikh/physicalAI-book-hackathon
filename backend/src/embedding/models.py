from dataclasses import dataclass
from datetime import datetime
from typing import List, Dict, Any, Optional


@dataclass
class EmbeddingVector:
    """
    Represents a vector embedding of a text chunk, suitable for semantic search.

    Attributes:
        embedding_id: Unique identifier for the embedding
        chunk_id: Reference to the source text chunk
        vector: The actual embedding vector (1024-dimensional for Cohere v3)
        model_used: Name of the embedding model used
        created_at: Timestamp when the embedding was generated
        metadata: Additional metadata about the embedding
    """
    embedding_id: str
    chunk_id: str
    vector: List[float]
    model_used: str
    created_at: datetime = None
    metadata: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Initialize default values after creation."""
        if self.created_at is None:
            self.created_at = datetime.now()
        if self.metadata is None:
            self.metadata = {}

    def validate(self) -> bool:
        """
        Validate the embedding vector.

        Returns:
            True if the embedding is valid, False otherwise
        """
        if not self.embedding_id:
            raise ValueError("embedding_id is required")
        if not self.chunk_id:
            raise ValueError("chunk_id is required")
        if not self.vector:
            raise ValueError("vector is required")
        if not self.model_used:
            raise ValueError("model_used is required")

        # Check that vector has correct dimensions (1024 for Cohere v3)
        if len(self.vector) != 1024:
            raise ValueError(f"Vector must have 1024 dimensions for Cohere v3, got {len(self.vector)}")

        # Check for NaN or infinity values
        for val in self.vector:
            if not isinstance(val, (int, float)) or val != val:  # Check for NaN using val != val
                raise ValueError("Vector contains invalid values (NaN or infinity)")

        return True

    @classmethod
    def from_generator_result(cls, result: Dict[str, Any]) -> 'EmbeddingVector':
        """
        Create an EmbeddingVector instance from a generator result.

        Args:
            result: Dictionary containing embedding data from the generator

        Returns:
            EmbeddingVector instance
        """
        return cls(
            embedding_id=result.get('embedding_id', result.get('chunk_id', f"emb_{hash(str(result.get('vector', [])))}")),
            chunk_id=result['chunk_id'],
            vector=result['vector'],
            model_used=result.get('model_used', 'unknown'),
            created_at=result.get('created_at', datetime.now()),
            metadata=result.get('metadata', {
                'document_id': result.get('document_id'),
                'source_url': result.get('source_url', ''),
                'chunk_number': result.get('chunk_number'),
                'token_count': result.get('token_count'),
                'generation_method': 'cohere_api'
            })
        )

    def to_serializable(self) -> Dict[str, Any]:
        """
        Convert the embedding to a serializable format.

        Returns:
            Dictionary representation of the embedding
        """
        return {
            'embedding_id': self.embedding_id,
            'chunk_id': self.chunk_id,
            'vector': self.vector,
            'model_used': self.model_used,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'metadata': self.metadata
        }