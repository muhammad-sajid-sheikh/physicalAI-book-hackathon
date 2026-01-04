#!/usr/bin/env python3
"""
Script to extract URLs from the Physical AI Book website and prepare for ingestion.
"""

import requests
from bs4 import BeautifulSoup
import logging
import sys
import os
from urllib.parse import urljoin, urlparse

# Configure basic logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def extract_urls_from_sitemap(sitemap_url: str) -> list:
    """
    Extract URLs from a sitemap XML file.

    Args:
        sitemap_url: URL of the sitemap.xml file

    Returns:
        List of URLs extracted from the sitemap
    """
    try:
        logger.info(f"Fetching sitemap from {sitemap_url}")
        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; DocusaurusBot/1.0)'
        }
        response = requests.get(sitemap_url, headers=headers)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'xml')  # Use 'xml' parser for sitemaps
        urls = []

        # Look for <url><loc>URL</loc></url> patterns in sitemap
        for url_tag in soup.find_all('url'):
            loc_tag = url_tag.find('loc')
            if loc_tag:
                url = loc_tag.text.strip()
                # Filter out non-document URLs if needed
                if url and url.startswith('http'):
                    # Skip certain URL patterns that are not content pages
                    if not any(skip in url.lower() for skip in ['.jpg', '.png', '.pdf', '.zip', 'assets/', 'static/']):
                        urls.append(url)

        # If no URLs found with 'url' tag, try 'sitemap' for nested sitemaps
        if not urls:
            for sitemap_tag in soup.find_all('sitemap'):
                loc_tag = sitemap_tag.find('loc')
                if loc_tag:
                    nested_sitemap_url = loc_tag.text.strip()
                    logger.info(f"Found nested sitemap: {nested_sitemap_url}")
                    # Recursively get URLs from nested sitemap
                    nested_urls = extract_urls_from_sitemap(nested_sitemap_url)
                    urls.extend(nested_urls)

        logger.info(f"Extracted {len(urls)} URLs from sitemap")
        return urls

    except Exception as e:
        logger.error(f"Error extracting URLs from sitemap: {str(e)}")
        return []


def main():
    """Main function to extract URLs from the sitemap."""
    logger.info("Starting URL extraction from Physical AI Book sitemap...")

    # The sitemap URL provided
    sitemap_url = "https://physical-ai-book-hackathon-mauve.vercel.app/sitemap.xml"

    # Extract URLs from the sitemap
    urls = extract_urls_from_sitemap(sitemap_url)

    if urls:
        logger.info(f"Found {len(urls)} URLs to process:")

        # Print the URLs
        for i, url in enumerate(urls, 1):
            print(f"{i:3d}. {url}")

        print(f"\nTotal URLs found: {len(urls)}")

        # Save URLs to a file for reference
        with open("extracted_urls.txt", "w") as f:
            for url in urls:
                f.write(f"{url}\n")

        print(f"\nURLs saved to 'extracted_urls.txt'")

        print("\n" + "="*60)
        print("TO RUN THE INGESTION PIPELINE:")
        print("="*60)
        print("1. Get your API keys:")
        print("   - Cohere API key (for embeddings)")
        print("   - Qdrant API key and URL (for vector storage)")
        print()
        print("2. Update the .env file with your actual API keys:")
        print("   COHERE_API_KEY=your_actual_cohere_key")
        print("   QDRANT_URL=your_actual_qdrant_url")
        print("   QDRANT_API_KEY=your_actual_qdrant_key")
        print()
        print("3. Run the main pipeline with the extracted URLs:")
        print("   python main.py <url1> <url2> <url3> ...")
        print()
        print("4. Or run the ingestion script:")
        print("   python ingest_from_sitemap.py")
        print()
        print("Note: The pipeline will process all extracted URLs and store")
        print("      embeddings in Qdrant with proper metadata.")

    else:
        logger.warning("No URLs found in the sitemap.")
        print("No URLs found. The sitemap might be empty or inaccessible.")
        print("Try accessing the URL directly in a browser to verify it exists.")


if __name__ == "__main__":
    main()