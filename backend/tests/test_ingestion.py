"""Unit tests for the ingestion components of the Docusaurus Content Ingestion Pipeline."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import requests
from bs4 import BeautifulSoup
from src.ingestion.url_fetcher import URLFetcher
from src.ingestion.processor import DocumentProcessor
from src.ingestion.models import DocumentContent


class TestURLFetcher(unittest.TestCase):
    """Test cases for the URLFetcher class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.fetcher = URLFetcher()

    def test_initialization(self):
        """Test that URLFetcher initializes correctly with default settings."""
        self.assertIsNotNone(self.fetcher.session)
        self.assertEqual(self.fetcher.session.headers['User-Agent'], 'Mozilla/5.0 (compatible; DocusaurusBot/1.0)')
        self.assertIsInstance(self.fetcher.timeout, int)

    @patch('src.ingestion.url_fetcher.requests.Session.get')
    def test_fetch_content_success(self, mock_get):
        """Test successful content fetching."""
        # Mock response
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.content = b'<html><head><title>Test Title</title></head><body><div class="markdown">Test content</div></body></html>'
        mock_get.return_value = mock_response

        # Call the method
        result = self.fetcher.fetch_content('https://example.com')

        # Verify the result
        self.assertEqual(result['url'], 'https://example.com')
        self.assertEqual(result['title'], 'Test Title')
        self.assertIn('Test content', result['content'])

    @patch('src.ingestion.url_fetcher.requests.Session.get')
    def test_fetch_content_with_missing_title(self, mock_get):
        """Test content fetching when title is missing."""
        # Mock response without title
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.content = b'<html><body><div class="markdown">Test content</div></body></html>'
        mock_get.return_value = mock_response

        # Call the method
        result = self.fetcher.fetch_content('https://example.com')

        # Verify the result uses URL as title when no title is present
        self.assertEqual(result['url'], 'https://example.com')
        self.assertEqual(result['title'], 'https://example.com')

    @patch('src.ingestion.url_fetcher.requests.Session.get')
    def test_fetch_content_request_exception(self, mock_get):
        """Test that request exceptions are properly handled."""
        # Configure mock to raise an exception
        mock_get.side_effect = requests.RequestException("Connection error")

        # Verify that the exception is raised
        with self.assertRaises(requests.RequestException):
            self.fetcher.fetch_content('https://example.com')

    @patch('src.ingestion.url_fetcher.requests.Session.get')
    def test_fetch_content_no_main_content(self, mock_get):
        """Test that appropriate error is raised when no main content is found."""
        # Mock response with no main content
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.content = b'<html><head><title>Test</title></head><body><p>Some other content</p></body></html>'
        mock_get.return_value = mock_response

        # Verify that ValueError is raised
        with self.assertRaises(ValueError) as context:
            self.fetcher.fetch_content('https://example.com')

        self.assertIn("Could not find main content", str(context.exception))

    def test_extract_main_content(self):
        """Test the _extract_main_content method with various selectors."""
        html_content = '<html><body><div class="markdown">Test content</div></body></html>'
        soup = BeautifulSoup(html_content, 'html.parser')

        result = self.fetcher._extract_main_content(soup)
        self.assertIsNotNone(result)
        self.assertIn('Test content', result.get_text())

    def test_clean_content(self):
        """Test the _clean_content method removes unwanted elements."""
        html_content = '<html><body><nav>Navigation</nav><div class="markdown">Test content</div><footer>Footer</footer></body></html>'
        soup = BeautifulSoup(html_content, 'html.parser')

        self.fetcher._clean_content(soup)

        # Check that navigation and footer elements are removed
        nav_elements = soup.find_all('nav')
        footer_elements = soup.find_all('footer')

        self.assertEqual(len(nav_elements), 0)
        self.assertEqual(len(footer_elements), 0)


class TestDocumentProcessor(unittest.TestCase):
    """Test cases for the DocumentProcessor class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.processor = DocumentProcessor()

    def test_initialization(self):
        """Test that DocumentProcessor initializes correctly."""
        self.assertIsNotNone(self.processor.fetcher)
        self.assertIsNotNone(self.processor.chunker)
        self.assertIsNotNone(self.processor.cleaner)
        self.assertEqual(self.processor.chunker.chunk_size_tokens, 512)  # Default value

    @patch('src.ingestion.processor.URLFetcher.fetch_content')
    def test_fetch_and_extract_success(self, mock_fetch_content):
        """Test successful fetch and extract operation."""
        # Mock the fetch_content method
        mock_fetch_content.return_value = {
            'url': 'https://example.com',
            'title': 'Test Title',
            'content': 'Test content',
            'html': '<div>Test content</div>'
        }

        # Call the method
        document = self.processor.fetch_and_extract('https://example.com')

        # Verify the result
        self.assertIsInstance(document, DocumentContent)
        self.assertEqual(document.source_url, 'https://example.com')
        self.assertEqual(document.title, 'Test Title')
        self.assertEqual(document.content, 'Test content')

    @patch('src.ingestion.processor.URLFetcher.fetch_content')
    def test_fetch_and_extract_with_exception(self, mock_fetch_content):
        """Test that exceptions during fetch are properly handled."""
        # Configure mock to raise an exception
        mock_fetch_content.side_effect = requests.RequestException("Network error")

        # Verify that the exception is propagated
        with self.assertRaises(requests.RequestException):
            self.processor.fetch_and_extract('https://example.com')

    def test_chunk_content(self):
        """Test content chunking functionality."""
        # Create a test document with substantial content
        test_content = "This is a test document. " * 100  # Create content longer than default chunk size
        document = DocumentContent(
            source_url='https://example.com',
            title='Test Document',
            content=test_content,
            html_content='<div>' + test_content + '</div>',
            metadata={}
        )

        # Chunk the content
        chunks = self.processor.chunk_content(document)

        # Verify that chunks were created
        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIsNotNone(chunk.content)
            self.assertIsNotNone(chunk.chunk_id)
            self.assertIsNotNone(chunk.document_id)

    def test_process_multiple_urls_success(self):
        """Test processing multiple URLs successfully."""
        with patch.object(self.processor, 'fetch_and_extract') as mock_fetch:
            # Mock successful fetch for multiple URLs
            mock_fetch.side_effect = [
                DocumentContent(
                    source_url='https://example1.com',
                    title='Doc 1',
                    content='Content 1',
                    html_content='<div>Content 1</div>',
                    metadata={}
                ),
                DocumentContent(
                    source_url='https://example2.com',
                    title='Doc 2',
                    content='Content 2',
                    html_content='<div>Content 2</div>',
                    metadata={}
                )
            ]

            # Process multiple URLs
            documents = self.processor.process_multiple_urls([
                'https://example1.com',
                'https://example2.com'
            ])

            # Verify results
            self.assertEqual(len(documents), 2)
            self.assertEqual(documents[0].title, 'Doc 1')
            self.assertEqual(documents[1].title, 'Doc 2')

    def test_process_multiple_urls_with_failure(self):
        """Test processing multiple URLs with some failures."""
        with patch.object(self.processor, 'fetch_and_extract') as mock_fetch:
            # Mock one success and one failure
            mock_fetch.side_effect = [
                DocumentContent(
                    source_url='https://example1.com',
                    title='Doc 1',
                    content='Content 1',
                    html_content='<div>Content 1</div>',
                    metadata={}
                ),
                Exception("Network error")  # This simulates a failure for the second URL
            ]

            # Process multiple URLs
            documents = self.processor.process_multiple_urls([
                'https://example1.com',
                'https://example2.com'
            ])

            # Verify that only the successful document is returned
            self.assertEqual(len(documents), 1)
            self.assertEqual(documents[0].title, 'Doc 1')


class TestDocumentContent(unittest.TestCase):
    """Test cases for the DocumentContent class."""

    def test_document_creation(self):
        """Test creating a DocumentContent instance."""
        document = DocumentContent(
            source_url='https://example.com',
            title='Test Title',
            content='Test content',
            html_content='<div>Test content</div>',
            metadata={'author': 'Test Author'}
        )

        self.assertEqual(document.source_url, 'https://example.com')
        self.assertEqual(document.title, 'Test Title')
        self.assertEqual(document.content, 'Test content')
        self.assertEqual(document.metadata['author'], 'Test Author')

    def test_document_validation(self):
        """Test document validation."""
        document = DocumentContent(
            source_url='https://example.com',
            title='Test Title',
            content='Test content',
            html_content='<div>Test content</div>',
            metadata={}
        )

        # Validation should pass for valid document
        self.assertTrue(document.validate())

    def test_document_validation_empty_content(self):
        """Test document validation with empty content."""
        document = DocumentContent(
            source_url='https://example.com',
            title='Test Title',
            content='',
            html_content='',
            metadata={}
        )

        # Validation should fail for empty content
        with self.assertRaises(ValueError):
            document.validate()


if __name__ == '__main__':
    unittest.main()