import re
from typing import List, Dict, Any


class TextChunker:
    """
    Handles the chunking of text content into segments that preserve semantic boundaries.
    """

    def __init__(self, chunk_size_tokens: int = 512, overlap_percentage: float = 0.2):
        """
        Initialize the TextChunker.

        Args:
            chunk_size_tokens: Target size for text chunks in tokens
            overlap_percentage: Percentage overlap between chunks
        """
        self.chunk_size_tokens = chunk_size_tokens
        self.overlap_percentage = overlap_percentage
        self.overlap_size = int(chunk_size_tokens * overlap_percentage)

    def chunk_text(self, text: str) -> List[Dict[str, Any]]:
        """
        Chunk text into segments preserving semantic boundaries.

        Args:
            text: Text to be chunked

        Returns:
            List of dictionaries containing chunk content and metadata
        """
        if not text.strip():
            return []

        # Split text into sentences as potential chunk boundaries
        sentences = re.split(r'[.!?]+\s+', text)

        chunks = []
        current_chunk = ""
        chunk_start = 0
        chunk_number = 0

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            # Estimate token count (rough approximation: 1 token ~ 4 chars)
            current_tokens = len(current_chunk.split()) if current_chunk else 0
            sentence_tokens = len(sentence.split())

            # Check if adding this sentence would exceed chunk size
            if current_tokens + sentence_tokens > self.chunk_size_tokens and current_chunk:
                # Complete current chunk and start new one
                chunks.append({
                    'content': current_chunk.strip(),
                    'start_offset': chunk_start,
                    'end_offset': chunk_start + len(current_chunk),
                    'chunk_number': chunk_number,
                    'token_count': current_tokens
                })

                # Start overlapping chunk - take some content from the end of the previous chunk
                overlap_start_idx = max(0, len(current_chunk) - int(self.overlap_size * 4))  # Approximate char overlap
                current_chunk = current_chunk[overlap_start_idx:] + " " + sentence
                chunk_start = len(text[:chunk_start + len(current_chunk)]) - len(current_chunk)
                chunk_number += 1
            else:
                current_chunk += " " + sentence

        # Add the last chunk if it contains content
        if current_chunk.strip():
            final_tokens = len(current_chunk.split())
            chunks.append({
                'content': current_chunk.strip(),
                'start_offset': chunk_start,
                'end_offset': chunk_start + len(current_chunk),
                'chunk_number': chunk_number,
                'token_count': final_tokens
            })

        return chunks

    def estimate_tokens(self, text: str) -> int:
        """
        Estimate the number of tokens in a text string.

        Args:
            text: Text to estimate tokens for

        Returns:
            Estimated number of tokens
        """
        # Simple heuristic: 1 token is roughly 4 characters
        # This is a rough approximation - for more accuracy, use tiktoken or similar
        return len(text.split())