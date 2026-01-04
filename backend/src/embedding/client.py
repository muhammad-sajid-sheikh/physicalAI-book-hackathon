import cohere
from typing import List, Dict, Any
import logging
from ..config import get_config

class CohereClient:
    """
    Client for interacting with the Cohere API for generating embeddings.
    """

    def __init__(self, api_key: str = None):
        """
        Initialize the Cohere client.

        Args:
            api_key: Cohere API key. If not provided, will try to read from configuration.
        """
        # Load configuration
        config = get_config()

        # Use provided API key or fall back to config
        cohere_api_key = api_key if api_key is not None else config.api.cohere_api_key

        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY must be set in configuration")

        self.client = cohere.Client(cohere_api_key)
        self.model = "embed-english-v3.0"  # Using the latest English model
        self.logger = logging.getLogger(__name__)

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to generate embeddings for
            input_type: Type of input (e.g., "search_document", "search_query", "classification", "clustering")

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            # Cohere has a maximum batch size of 96, so we need to process in chunks if needed
            batch_size = 96
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type=input_type
                )

                # Extract embeddings from the response
                batch_embeddings = response.embeddings
                all_embeddings.extend(batch_embeddings)

            self.logger.info(f"Successfully generated embeddings for {len(texts)} texts")
            return all_embeddings
        except Exception as e:
            self.logger.error(f"Error generating embeddings: {str(e)}")
            raise

    def generate_embedding(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for
            input_type: Type of input (e.g., "search_document", "search_query", "classification", "clustering")

        Returns:
            Embedding as a list of floats
        """
        try:
            embeddings = self.generate_embeddings([text], input_type)
            return embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            self.logger.error(f"Error generating embedding for text: {str(e)}")
            raise

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the current embedding model.

        Returns:
            Dictionary with model information
        """
        return {
            'model_name': self.model,
            'dimensions': 1024,  # Cohere v3 English model has 1024 dimensions
            'input_type_options': ['search_document', 'search_query', 'classification', 'clustering']
        }