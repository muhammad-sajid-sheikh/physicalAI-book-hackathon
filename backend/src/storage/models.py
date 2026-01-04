from dataclasses import dataclass
from datetime import datetime
from typing import List, Dict, Any, Optional


@dataclass
class VectorRecord:
    """
    Represents a complete searchable record stored in Qdrant, containing a vector and its metadata.

    Attributes:
        record_id: Unique identifier for the Qdrant record
        vector: The actual embedding vector (1024-dimensional for Cohere v3)
        payload: Dictionary containing metadata associated with the vector
        created_at: Timestamp when the record was created
    """
    record_id: str
    vector: List[float]
    payload: Dict[str, Any]
    created_at: datetime = None

    def __post_init__(self):
        """Initialize default values after creation."""
        if self.created_at is None:
            self.created_at = datetime.now()

    def validate(self) -> bool:
        """
        Validate the vector record.

        Returns:
            True if the record is valid, False otherwise
        """
        if not self.record_id:
            raise ValueError("record_id is required")
        if not self.vector:
            raise ValueError("vector is required")
        if not self.payload:
            raise ValueError("payload is required")

        # Check that vector has correct dimensions (1024 for Cohere v3)
        if len(self.vector) != 1024:
            raise ValueError(f"Vector must have 1024 dimensions for Cohere v3, got {len(self.vector)}")

        # Check for NaN or infinity values
        for val in self.vector:
            if not isinstance(val, (int, float)) or val != val:  # Check for NaN using val != val
                raise ValueError("Vector contains invalid values (NaN or infinity)")

        return True

    @classmethod
    def from_embedding_data(cls, embedding_data: Dict[str, Any]) -> 'VectorRecord':
        """
        Create a VectorRecord instance from embedding data.

        Args:
            embedding_data: Dictionary containing embedding and metadata

        Returns:
            VectorRecord instance
        """
        return cls(
            record_id=embedding_data.get('embedding_id', embedding_data.get('chunk_id', f"record_{hash(str(embedding_data.get('vector', [])))}")),
            vector=embedding_data['vector'],
            payload={
                'source_url': embedding_data.get('source_url', ''),
                'document_title': embedding_data.get('document_title', ''),
                'chunk_content': embedding_data.get('content', ''),
                'chunk_id': embedding_data.get('chunk_id', ''),
                'document_id': embedding_data.get('document_id', ''),
                'chunk_number': embedding_data.get('chunk_number', 0),
                'model_used': embedding_data.get('model_used', ''),
                'token_count': embedding_data.get('token_count', 0),
                'created_at': embedding_data.get('created_at', datetime.now().isoformat()),
                'metadata': embedding_data.get('metadata', {})
            }
        )

    def to_qdrant_payload(self) -> Dict[str, Any]:
        """
        Convert the vector record to a format suitable for Qdrant storage.

        Returns:
            Dictionary in Qdrant payload format
        """
        return {
            'source_url': self.payload.get('source_url', ''),
            'document_title': self.payload.get('document_title', ''),
            'chunk_content': self.payload.get('chunk_content', ''),
            'chunk_id': self.payload.get('chunk_id', ''),
            'document_id': self.payload.get('document_id', ''),
            'chunk_number': self.payload.get('chunk_number', 0),
            'model_used': self.payload.get('model_used', ''),
            'token_count': self.payload.get('token_count', 0),
            'created_at': self.payload.get('created_at', self.created_at.isoformat()),
            'metadata': self.payload.get('metadata', {})
        }

    def get_searchable_content(self) -> str:
        """
        Get the content that should be used for semantic search.

        Returns:
            String content for semantic search
        """
        return self.payload.get('chunk_content', '')