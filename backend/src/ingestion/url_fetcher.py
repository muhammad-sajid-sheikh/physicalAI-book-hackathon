import requests
from bs4 import BeautifulSoup
from typing import Dict, Any
import logging
from ..config import get_config

class URLFetcher:
    def __init__(self, timeout: int = None):
        # Load configuration
        config = get_config()

        # Use provided timeout or fall back to config
        request_timeout = timeout if timeout is not None else config.processing.timeout

        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; DocusaurusBot/1.0)'
        })
        self.timeout = request_timeout
        self.logger = logging.getLogger(__name__)

    def fetch_content(self, url: str) -> Dict[str, Any]:
        """
        Fetch and extract content from a Docusaurus URL.

        Args:
            url: The URL to fetch content from

        Returns:
            Dictionary containing the URL, title, content, and HTML
        """
        try:
            response = self.session.get(url, timeout=self.timeout)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Extract main content (common Docusaurus selectors)
            main_content = self._extract_main_content(soup)

            if not main_content:
                raise ValueError(f"Could not find main content in {url}")

            # Clean up the content (remove navigation, etc.)
            self._clean_content(soup)

            title = soup.find('title')
            title_text = title.text if title else url

            return {
                'url': url,
                'title': title_text,
                'content': main_content.get_text(strip=True),
                'html': str(main_content)
            }
        except requests.RequestException as e:
            self.logger.error(f"Error fetching content from {url}: {str(e)}")
            raise
        except Exception as e:
            self.logger.error(f"Error processing content from {url}: {str(e)}")
            raise

    def _extract_main_content(self, soup: BeautifulSoup) -> BeautifulSoup:
        """
        Extract main content from the soup using common Docusaurus selectors.

        Args:
            soup: BeautifulSoup object with the page content

        Returns:
            BeautifulSoup object representing the main content
        """
        # Try various common Docusaurus content selectors in order of preference
        selectors = [
            # Docusaurus v2/v3 markdown content
            {'class_': 'markdown'},
            {'class_': 'container markdown'},
            {'class_': 'theme-doc-markdown'},
            # Generic content containers
            {'name': 'article'},
            {'name': 'main'},
            {'attrs': {'role': 'main'}},
            {'class_': 'container'},
            # Content wrapper classes
            {'class_': 'docPage_'},
            {'class_': 'docMainContainer_'},
            {'class_': 'main-wrapper'},
            {'class_': 'main-content'},
        ]

        for selector in selectors:
            if 'class_' in selector:
                element = soup.find(class_=selector['class_'])
            elif 'name' in selector:
                element = soup.find(selector['name'])
            elif 'attrs' in selector:
                element = soup.find(attrs=selector['attrs'])
            else:
                continue

            if element:
                return element

        # If no specific selector matched, return the body
        body = soup.find('body')
        return body if body else soup

    def _clean_content(self, soup: BeautifulSoup) -> None:
        """
        Remove navigation elements and other non-content elements from the soup.

        Args:
            soup: BeautifulSoup object to clean
        """
        # Elements to remove (navigation, headers, footers, etc.)
        elements_to_remove = [
            # Navigation elements
            {'name': 'nav'},
            {'name': 'aside'},
            {'class_': 'navbar'},
            {'class_': 'sidebar'},
            {'class_': 'menu'},
            {'class_': 'header'},
            {'class_': 'footer'},
            {'class_': 'toc'},
            {'class_': 'table-of-contents'},
            # Ads and banners
            {'class_': 'ad'},
            {'class_': 'advertisement'},
            {'class_': 'banner'},
            # Other non-content elements
            {'class_': 'pagination'},
            {'class_': 'comments'},
            {'class_': 'social-share'},
        ]

        for selector in elements_to_remove:
            if 'class_' in selector:
                for element in soup.find_all(class_=selector['class_']):
                    element.decompose()
            elif 'name' in selector:
                for element in soup.find_all(selector['name']):
                    element.decompose()

        # Remove script and style elements
        for element in soup.find_all(['script', 'style']):
            element.decompose()