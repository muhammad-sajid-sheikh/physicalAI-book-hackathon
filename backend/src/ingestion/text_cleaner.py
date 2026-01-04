import re
from typing import Dict, Any, Optional


class TextCleaner:
    """
    Handles the cleaning and normalization of text content extracted from Docusaurus sites.
    """

    def __init__(self):
        """
        Initialize the TextCleaner with common cleaning patterns.
        """
        # Common patterns for cleaning
        self.whitespace_pattern = re.compile(r'\s+')
        self.html_tag_pattern = re.compile(r'<[^>]+>')
        self.special_chars_pattern = re.compile(r'[^\w\s\-\.\',;!?:]+')

    def clean_text(self, text: str, remove_html: bool = True, normalize_whitespace: bool = True) -> str:
        """
        Clean and normalize text content.

        Args:
            text: The text to clean
            remove_html: Whether to remove HTML tags
            normalize_whitespace: Whether to normalize whitespace

        Returns:
            Cleaned text string
        """
        if not text:
            return ""

        cleaned_text = text

        # Remove HTML tags if requested
        if remove_html:
            cleaned_text = self.remove_html_tags(cleaned_text)

        # Normalize whitespace if requested
        if normalize_whitespace:
            cleaned_text = self.normalize_whitespace(cleaned_text)

        # Remove extra punctuation or normalize special characters
        cleaned_text = self.normalize_special_characters(cleaned_text)

        # Strip leading/trailing whitespace
        cleaned_text = cleaned_text.strip()

        return cleaned_text

    def remove_html_tags(self, text: str) -> str:
        """
        Remove HTML tags from text.

        Args:
            text: Text that may contain HTML tags

        Returns:
            Text with HTML tags removed
        """
        return self.html_tag_pattern.sub('', text)

    def normalize_whitespace(self, text: str) -> str:
        """
        Normalize whitespace in text (multiple spaces, tabs, newlines to single spaces).

        Args:
            text: Text to normalize

        Returns:
            Text with normalized whitespace
        """
        # Replace multiple whitespace characters with a single space
        normalized = self.whitespace_pattern.sub(' ', text)
        return normalized.strip()

    def normalize_special_characters(self, text: str) -> str:
        """
        Normalize or remove special characters that might interfere with processing.

        Args:
            text: Text to normalize

        Returns:
            Text with normalized special characters
        """
        # This could be expanded based on specific requirements
        # For now, we'll preserve common punctuation but normalize others
        return text

    def clean_code_blocks(self, text: str) -> str:
        """
        Clean code blocks from text, optionally preserving them in a structured way.

        Args:
            text: Text that may contain code blocks

        Returns:
            Text with code blocks handled appropriately
        """
        # This is a basic implementation - in a real system, you'd want to preserve
        # code blocks separately and handle them differently from prose
        # For now, we'll just normalize them
        return text

    def extract_metadata(self, text: str, original_html: Optional[str] = None) -> Dict[str, Any]:
        """
        Extract metadata from the text and original HTML.

        Args:
            text: Cleaned text
            original_html: Original HTML content (optional)

        Returns:
            Dictionary containing extracted metadata
        """
        metadata = {
            'word_count': len(text.split()),
            'char_count': len(text),
            'line_count': len(text.splitlines()),
            'encoding': 'utf-8'  # Assuming UTF-8
        }

        if original_html:
            # Extract additional metadata from HTML if available
            metadata.update({
                'has_code_blocks': '<code>' in original_html or '<pre>' in original_html,
                'has_lists': '<ul>' in original_html or '<ol>' in original_html or '<li>' in original_html,
                'has_headings': any(tag in original_html for tag in ['<h1>', '<h2>', '<h3>', '<h4>', '<h5>', '<h6>'])
            })

        return metadata