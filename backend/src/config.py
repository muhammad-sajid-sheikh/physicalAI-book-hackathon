"""Configuration management for the Docusaurus Content Ingestion Pipeline."""

import os
from dataclasses import dataclass
from typing import Optional
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


@dataclass
class ProcessingConfig:
    """
    Configuration for processing parameters.

    Attributes:
        chunk_size_tokens: Target size for text chunks in tokens (default 512)
        overlap_percentage: Overlap percentage between chunks (default 0.2)
        batch_size: Number of items to process in each batch (default 100)
        max_retries: Maximum number of retry attempts for API calls (default 3)
        timeout: Request timeout in seconds (default 30)
    """
    chunk_size_tokens: int = 512
    overlap_percentage: float = 0.2
    batch_size: int = 100
    max_retries: int = 3
    timeout: int = 30

    def __post_init__(self):
        """Validate configuration values after initialization."""
        self.validate()

    def validate(self) -> bool:
        """
        Validate the configuration values.

        Returns:
            True if configuration is valid, raises ValueError otherwise
        """
        if self.chunk_size_tokens <= 0:
            raise ValueError(f"chunk_size_tokens must be positive, got {self.chunk_size_tokens}")

        if not 0 <= self.overlap_percentage <= 1:
            raise ValueError(f"overlap_percentage must be between 0 and 1, got {self.overlap_percentage}")

        if self.batch_size <= 0:
            raise ValueError(f"batch_size must be positive, got {self.batch_size}")

        if self.max_retries < 0:
            raise ValueError(f"max_retries must be non-negative, got {self.max_retries}")

        if self.timeout <= 0:
            raise ValueError(f"timeout must be positive, got {self.timeout}")

        return True


@dataclass
class APIConfig:
    """
    Configuration for API connections.

    Attributes:
        cohere_api_key: API key for Cohere services
        qdrant_url: URL for Qdrant vector database
        qdrant_api_key: API key for Qdrant services
        qdrant_collection_name: Name of the collection in Qdrant (default 'docs_embeddings')
    """
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = 'docs_embeddings'

    def __post_init__(self):
        """Validate API configuration values after initialization."""
        self.validate()

    def validate(self) -> bool:
        """
        Validate the API configuration values.

        Returns:
            True if configuration is valid, raises ValueError otherwise
        """
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY is required")

        if not self.qdrant_url:
            raise ValueError("QDRANT_URL is required")

        if not self.qdrant_api_key:
            raise ValueError("QDRANT_API_KEY is required")

        if not self.qdrant_collection_name:
            raise ValueError("QDRANT_COLLECTION_NAME is required")

        return True


@dataclass
class AppConfig:
    """
    Main application configuration combining processing and API configs.

    Attributes:
        processing: ProcessingConfig instance with processing parameters
        api: APIConfig instance with API connection details
    """
    processing: ProcessingConfig
    api: APIConfig

    def validate(self) -> bool:
        """
        Validate the entire application configuration.

        Returns:
            True if configuration is valid, raises ValueError otherwise
        """
        self.processing.validate()
        self.api.validate()
        return True


def load_config_from_env() -> AppConfig:
    """
    Load configuration from environment variables.

    Returns:
        AppConfig instance with values loaded from environment
    """
    # Load processing configuration from environment or use defaults
    chunk_size_tokens = int(os.getenv('CHUNK_SIZE_TOKENS', '512'))
    overlap_percentage = float(os.getenv('OVERLAP_PERCENTAGE', '0.2'))
    batch_size = int(os.getenv('BATCH_SIZE', '100'))
    max_retries = int(os.getenv('MAX_RETRIES', '3'))
    timeout = int(os.getenv('TIMEOUT', '30'))

    processing_config = ProcessingConfig(
        chunk_size_tokens=chunk_size_tokens,
        overlap_percentage=overlap_percentage,
        batch_size=batch_size,
        max_retries=max_retries,
        timeout=timeout
    )

    # Load API configuration from environment
    cohere_api_key = os.getenv('COHERE_API_KEY')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    qdrant_collection_name = os.getenv('QDRANT_COLLECTION_NAME', 'docs_embeddings')

    api_config = APIConfig(
        cohere_api_key=cohere_api_key,
        qdrant_url=qdrant_url,
        qdrant_api_key=qdrant_api_key,
        qdrant_collection_name=qdrant_collection_name
    )

    return AppConfig(
        processing=processing_config,
        api=api_config
    )


def validate_config(config: AppConfig) -> bool:
    """
    Validate the provided configuration.

    Args:
        config: AppConfig instance to validate

    Returns:
        True if configuration is valid, raises ValueError otherwise
    """
    return config.validate()


# Global configuration instance
_config: Optional[AppConfig] = None


def get_config() -> AppConfig:
    """
    Get the global configuration instance, loading from environment if not already loaded.

    Returns:
        AppConfig instance
    """
    global _config
    if _config is None:
        _config = load_config_from_env()
    return _config


def reset_config():
    """Reset the global configuration instance (useful for testing)."""
    global _config
    _config = None


if __name__ == "__main__":
    # Example usage
    logger = logging.getLogger(__name__)

    try:
        config = get_config()
        logger.info("Configuration loaded successfully")
        logger.info(f"Chunk size: {config.processing.chunk_size_tokens} tokens")
        logger.info(f"Overlap: {config.processing.overlap_percentage * 100}%")
        logger.info(f"Qdrant collection: {config.api.qdrant_collection_name}")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        raise