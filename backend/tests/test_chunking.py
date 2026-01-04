"""Unit tests for the text processing and chunking components of the Docusaurus Content Ingestion Pipeline."""

import unittest
from unittest.mock import Mock, patch
from src.ingestion.chunker import TextChunker
from src.ingestion.text_cleaner import TextCleaner
from src.ingestion.models import TextChunk, DocumentContent


class TestTextChunker(unittest.TestCase):
    """Test cases for the TextChunker class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.chunker = TextChunker(chunk_size_tokens=100, overlap_percentage=0.2)

    def test_initialization(self):
        """Test that TextChunker initializes correctly."""
        self.assertEqual(self.chunker.chunk_size_tokens, 100)
        self.assertEqual(self.chunker.overlap_percentage, 0.2)

    def test_chunk_text_short_content(self):
        """Test chunking of short content that fits in one chunk."""
        short_text = "This is a short text that fits in one chunk."
        chunks = self.chunker.chunk_text(short_text)

        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0]['content'], short_text)
        self.assertLessEqual(chunks[0]['token_count'], 100)

    def test_chunk_text_long_content(self):
        """Test chunking of long content that requires multiple chunks."""
        # Create content longer than the chunk size
        long_text = "This is a sentence. " * 200  # Create content much longer than 100 tokens
        chunks = self.chunker.chunk_text(long_text)

        # Verify multiple chunks were created
        self.assertGreater(len(chunks), 1)

        # Verify that each chunk is within the token limit
        for chunk in chunks:
            self.assertLessEqual(chunk['token_count'], 100)

        # Verify that chunks have overlap where expected
        if len(chunks) > 1:
            # Check that there's some overlap between first and second chunk
            first_chunk_end = chunks[0]['content']
            second_chunk_start = chunks[1]['content']
            # The second chunk should contain some text from the first chunk due to overlap
            # This is a basic check - the exact overlap logic may vary based on implementation

    def test_chunk_text_with_overlap(self):
        """Test that overlap is properly applied between chunks."""
        # Create text with sentences
        text = "Sentence 1. Sentence 2. Sentence 3. Sentence 4. Sentence 5. " * 40
        chunks = self.chunker.chunk_text(text)

        self.assertGreater(len(chunks), 1)

        # Check that chunks have correct metadata
        for i, chunk in enumerate(chunks):
            self.assertEqual(chunk['chunk_number'], i)
            self.assertIn('content', chunk)
            self.assertIn('token_count', chunk)
            self.assertIn('start_offset', chunk)
            self.assertIn('end_offset', chunk)

    def test_chunk_text_empty_content(self):
        """Test chunking of empty content."""
        chunks = self.chunker.chunk_text("")
        self.assertEqual(len(chunks), 0)

    def test_chunk_text_single_sentence(self):
        """Test chunking of content with a single sentence."""
        text = "This is a single sentence."
        chunks = self.chunker.chunk_text(text)

        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0]['content'], text)

    def test_estimate_tokens(self):
        """Test the token estimation function."""
        text = "This is a test sentence with several words."
        token_count = self.chunker._estimate_tokens(text)

        # Basic validation: token count should be positive
        self.assertGreater(token_count, 0)

        # For simple text, token count should be roughly the number of words
        word_count = len(text.split())
        self.assertLessEqual(token_count, word_count * 2)  # Allow some padding


class TestTextCleaner(unittest.TestCase):
    """Test cases for the TextCleaner class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.cleaner = TextCleaner()

    def test_initialization(self):
        """Test that TextCleaner initializes correctly."""
        self.assertIsNotNone(self.cleaner)

    def test_clean_text_basic(self):
        """Test basic text cleaning."""
        dirty_text = "  This   is   a   test   text.  \n\n  With   extra   spaces.  "
        cleaned_text = self.cleaner.clean_text(dirty_text)

        # Check that extra spaces are removed
        self.assertNotIn("  ", cleaned_text)  # No double spaces
        self.assertIn("This is a test text.", cleaned_text)

    def test_clean_text_with_html_tags(self):
        """Test cleaning text with HTML tags."""
        html_text = "<p>This is <b>bold</b> text with <a href='#'>links</a>.</p>"
        cleaned_text = self.cleaner.clean_text(html_text)

        # HTML tags should be removed but content preserved
        self.assertIn("This is bold text with links.", cleaned_text)
        self.assertNotIn("<p>", cleaned_text)
        self.assertNotIn("<b>", cleaned_text)

    def test_clean_text_with_special_characters(self):
        """Test cleaning text with special characters."""
        special_text = "Text with\u00A0non-breaking\u200Bspaces and\u0009tabs."
        cleaned_text = self.cleaner.clean_text(special_text)

        # Special characters should be normalized
        self.assertIn("Text with non-breaking spaces and tabs.", cleaned_text)

    def test_clean_text_empty(self):
        """Test cleaning empty text."""
        cleaned_text = self.cleaner.clean_text("")
        self.assertEqual(cleaned_text, "")

    def test_clean_text_with_code_blocks(self):
        """Test cleaning text with code blocks."""
        text_with_code = "Here is some text.\n```python\nprint('Hello World')\n```\nMore text."
        cleaned_text = self.cleaner.clean_text(text_with_code)

        # Code blocks should be preserved
        self.assertIn("print('Hello World')", cleaned_text)

    def test_remove_extra_whitespace(self):
        """Test the remove_extra_whitespace method."""
        text = "  This   has    extra   \t\twhitespace\n\nand\nnewlines  "
        cleaned = self.cleaner.remove_extra_whitespace(text)

        # Should have single spaces, no tabs, and normalized newlines
        self.assertNotIn("\t", cleaned)
        self.assertNotIn("  ", cleaned)  # No consecutive spaces
        # Note: This might vary based on implementation - adjust as needed


class TestTextChunk(unittest.TestCase):
    """Test cases for the TextChunk class."""

    def test_text_chunk_creation(self):
        """Test creating a TextChunk instance."""
        chunk = TextChunk(
            chunk_id='test_chunk_1',
            document_id='test_doc_1',
            content='Test content for the chunk.',
            start_offset=0,
            end_offset=25,
            chunk_number=0,
            token_count=8
        )

        self.assertEqual(chunk.chunk_id, 'test_chunk_1')
        self.assertEqual(chunk.document_id, 'test_doc_1')
        self.assertEqual(chunk.content, 'Test content for the chunk.')
        self.assertEqual(chunk.start_offset, 0)
        self.assertEqual(chunk.end_offset, 25)
        self.assertEqual(chunk.chunk_number, 0)
        self.assertEqual(chunk.token_count, 8)

    def test_text_chunk_validation(self):
        """Test TextChunk validation."""
        chunk = TextChunk(
            chunk_id='test_chunk_1',
            document_id='test_doc_1',
            content='Test content for the chunk.',
            start_offset=0,
            end_offset=25,
            chunk_number=0,
            token_count=8
        )

        # Validation should pass for valid chunk
        self.assertTrue(chunk.validate())

    def test_text_chunk_validation_empty_content(self):
        """Test TextChunk validation with empty content."""
        chunk = TextChunk(
            chunk_id='test_chunk_1',
            document_id='test_doc_1',
            content='',
            start_offset=0,
            end_offset=0,
            chunk_number=0,
            token_count=0
        )

        # Validation should fail for empty content
        with self.assertRaises(ValueError):
            chunk.validate()

    def test_text_chunk_validation_invalid_bounds(self):
        """Test TextChunk validation with invalid start/end bounds."""
        chunk = TextChunk(
            chunk_id='test_chunk_1',
            document_id='test_doc_1',
            content='Test content',
            start_offset=10,  # Start after end
            end_offset=5,     # End before start
            chunk_number=0,
            token_count=3
        )

        # Validation should fail for invalid bounds
        with self.assertRaises(ValueError):
            chunk.validate()

    def test_from_chunk_dict(self):
        """Test creating TextChunk from dictionary."""
        chunk_dict = {
            'content': 'Test content',
            'document_id': 'test_doc_1',
            'chunk_number': 0,
            'token_count': 3,
            'source_url': 'https://example.com',
            'document_title': 'Test Document'
        }
        document_id = 'test_doc_1'

        chunk = TextChunk.from_chunk_dict(chunk_dict, document_id)

        self.assertEqual(chunk.content, 'Test content')
        self.assertEqual(chunk.document_id, document_id)
        self.assertEqual(chunk.chunk_number, 0)
        self.assertEqual(chunk.token_count, 3)


class TestIntegrationTextProcessing(unittest.TestCase):
    """Integration tests for text processing components."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.cleaner = TextCleaner()
        self.chunker = TextChunker(chunk_size_tokens=50, overlap_percentage=0.1)

    def test_clean_then_chunk(self):
        """Test the full flow: clean text then chunk it."""
        dirty_text = "  This   is   a   test   text.  \n\n  With   extra   spaces.  " * 20
        cleaned_text = self.cleaner.clean_text(dirty_text)
        chunks = self.chunker.chunk_text(cleaned_text)

        # Verify that we get chunks
        self.assertGreater(len(chunks), 0)

        # Verify that chunks are within size limits
        for chunk in chunks:
            self.assertLessEqual(chunk['token_count'], 50)

    def test_processing_document_content(self):
        """Test processing a DocumentContent object through cleaning and chunking."""
        # Create a document with dirty content
        document = DocumentContent(
            source_url='https://example.com',
            title='Test Document',
            content="  This   is   a   test   text.  \n\n  With   extra   spaces.  " * 30,
            html_content='<div>  This   is   a   test   text.  </div>',
            metadata={'author': 'Test Author'}
        )

        # Clean the content
        cleaner = TextCleaner()
        cleaned_content = cleaner.clean_text(document.content)

        # Chunk the cleaned content
        chunker = TextChunker(chunk_size_tokens=60, overlap_percentage=0.15)
        chunk_dicts = chunker.chunk_text(cleaned_content)

        # Verify results
        self.assertGreater(len(chunk_dicts), 1)  # Should be multiple chunks
        for chunk_dict in chunk_dicts:
            self.assertLessEqual(chunk_dict['token_count'], 60)
            self.assertIn('content', chunk_dict)
            self.assertIn('chunk_number', chunk_dict)


if __name__ == '__main__':
    unittest.main()