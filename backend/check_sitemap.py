#!/usr/bin/env python3
"""
Script to check the content of the sitemap.
"""

import requests
from bs4 import BeautifulSoup
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def check_sitemap(sitemap_url: str):
    """
    Check the content of a sitemap XML file.
    """
    try:
        logger.info(f"Fetching sitemap from {sitemap_url}")
        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; DocusaurusBot/1.0)'
        }
        response = requests.get(sitemap_url, headers=headers, timeout=30)
        response.raise_for_status()

        print("Sitemap content:")
        print(response.text[:2000])  # Print first 2000 characters
        print("...")

        soup = BeautifulSoup(response.content, 'xml')  # Use 'xml' parser for sitemaps

        # Look for <url><loc>URL</loc></url> patterns in sitemap
        urls = []
        for url_tag in soup.find_all('url'):
            loc_tag = url_tag.find('loc')
            if loc_tag:
                url = loc_tag.text.strip()
                urls.append(url)

        # Also look for nested sitemaps
        nested_sitemaps = []
        for sitemap_tag in soup.find_all('sitemap'):
            loc_tag = sitemap_tag.find('loc')
            if loc_tag:
                nested_sitemap = loc_tag.text.strip()
                nested_sitemaps.append(nested_sitemap)

        print(f"\nFound {len(urls)} URLs in main sitemap:")
        for i, url in enumerate(urls[:10]):  # Show first 10
            print(f"  {i+1}. {url}")
        if len(urls) > 10:
            print(f"  ... and {len(urls) - 10} more")

        print(f"\nFound {len(nested_sitemaps)} nested sitemaps:")
        for i, sitemap in enumerate(nested_sitemaps):
            print(f"  {i+1}. {sitemap}")

        # Check for physical-ai-book-hackathon-mauve.vercel.app URLs specifically
        main_site_urls = [url for url in urls if 'physical-ai-book-hackathon-mauve.vercel.app' in url]
        print(f"\nFound {len(main_site_urls)} URLs from main site:")
        for i, url in enumerate(main_site_urls):
            print(f"  {i+1}. {url}")

        return urls, nested_sitemaps

    except Exception as e:
        logger.error(f"Error checking sitemap: {str(e)}")
        import traceback
        traceback.print_exc()
        return [], []

if __name__ == "__main__":
    sitemap_url = "https://physical-ai-book-hackathon-mauve.vercel.app/sitemap.xml"
    urls, nested_sitemaps = check_sitemap(sitemap_url)