from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Any, Optional


@dataclass
class DocumentContent:
    """
    Represents extracted content from a Docusaurus documentation site.

    Attributes:
        source_url: The original URL from which content was extracted
        title: The title of the document
        content: The clean text content extracted from the page
        html_content: The original HTML content (optional)
        metadata: Additional metadata about the document
        created_at: Timestamp when the document was extracted
        processed_at: Timestamp when the document was processed (optional)
    """
    source_url: str
    title: str
    content: str
    html_content: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime = None
    processed_at: Optional[datetime] = None

    def __post_init__(self):
        """Initialize default values after creation."""
        if self.created_at is None:
            self.created_at = datetime.now()
        if self.metadata is None:
            self.metadata = {}

    def validate(self) -> bool:
        """
        Validate the document content.

        Returns:
            True if the document is valid, False otherwise
        """
        if not self.source_url:
            raise ValueError("source_url is required")
        if not self.title:
            raise ValueError("title is required")
        if not self.content:
            raise ValueError("content is required")

        # Check that content is not just whitespace
        if not self.content.strip():
            raise ValueError("content cannot be empty or just whitespace")

        return True

    @classmethod
    def from_url_fetcher_result(cls, fetcher_result: Dict[str, Any]) -> 'DocumentContent':
        """
        Create a DocumentContent instance from the result of URLFetcher.

        Args:
            fetcher_result: Result dictionary from URLFetcher.fetch_content()

        Returns:
            DocumentContent instance
        """
        return cls(
            source_url=fetcher_result['url'],
            title=fetcher_result['title'],
            content=fetcher_result['content'],
            html_content=fetcher_result.get('html'),
            metadata={
                'extracted_from': 'url_fetcher',
                'extraction_method': 'docusaurus_extraction',
                'extraction_timestamp': datetime.now().isoformat()
            }
        )


@dataclass
class TextChunk:
    """
    Represents a segmented portion of documentation content, optimally sized for vector transformation.

    Attributes:
        chunk_id: Unique identifier for the chunk
        document_id: Reference to the parent document
        content: The actual text content of the chunk
        start_offset: Character position where chunk starts in original document
        end_offset: Character position where chunk ends in original document
        chunk_number: Sequence number of the chunk in the document
        token_count: Number of tokens in the chunk
    """
    chunk_id: str
    document_id: str
    content: str
    start_offset: int
    end_offset: int
    chunk_number: int
    token_count: int

    def validate(self) -> bool:
        """
        Validate the text chunk.

        Returns:
            True if the chunk is valid, False otherwise
        """
        if not self.chunk_id:
            raise ValueError("chunk_id is required")
        if not self.document_id:
            raise ValueError("document_id is required")
        if not self.content:
            raise ValueError("content is required")
        if self.start_offset < 0:
            raise ValueError("start_offset must be non-negative")
        if self.end_offset < self.start_offset:
            raise ValueError("end_offset must be greater than or equal to start_offset")
        if self.chunk_number < 0:
            raise ValueError("chunk_number must be non-negative")
        if self.token_count < 0:
            raise ValueError("token_count must be non-negative")

        return True

    @classmethod
    def from_chunk_dict(cls, chunk_dict: Dict[str, Any], document_id: str) -> 'TextChunk':
        """
        Create a TextChunk instance from a chunk dictionary.

        Args:
            chunk_dict: Dictionary containing chunk data
            document_id: ID of the parent document

        Returns:
            TextChunk instance
        """
        return cls(
            chunk_id=chunk_dict.get('chunk_id', f"{document_id}_chunk_{chunk_dict.get('chunk_number', 0)}"),
            document_id=document_id,
            content=chunk_dict['content'],
            start_offset=chunk_dict.get('start_offset', 0),
            end_offset=chunk_dict.get('end_offset', len(chunk_dict['content'])),
            chunk_number=chunk_dict.get('chunk_number', 0),
            token_count=chunk_dict.get('token_count', len(chunk_dict['content'].split()))
        )