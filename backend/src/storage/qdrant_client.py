from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import logging
from ..embedding.models import EmbeddingVector
from ..config import get_config

class QdrantStorage:
    """
    Client for storing and retrieving embeddings in Qdrant vector database.
    """

    def __init__(self, url: str = None, api_key: str = None, collection_name: str = None):
        """
        Initialize the Qdrant client.

        Args:
            url: Qdrant cluster URL. If not provided, will try to read from configuration.
            api_key: Qdrant API key. If not provided, will try to read from configuration.
            collection_name: Name of the collection to use. If not provided, will use config value.
        """
        # Load configuration
        config = get_config()

        # Use provided values or fall back to config
        qdrant_url = url if url is not None else config.api.qdrant_url
        qdrant_api_key = api_key if api_key is not None else config.api.qdrant_api_key
        collection_name = collection_name if collection_name is not None else config.api.qdrant_collection_name

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in configuration")

        self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.collection_name = collection_name
        self.logger = logging.getLogger(__name__)

        # Create collection if it doesn't exist
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """Create Qdrant collection if it doesn't exist."""
        try:
            # Try to get the collection to see if it exists
            self.client.get_collection(self.collection_name)
            self.logger.info(f"Collection '{self.collection_name}' already exists")
        except Exception:
            # Collection doesn't exist, create it with 1024-dim vectors (for Cohere v3)
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
            )
            self.logger.info(f"Created collection '{self.collection_name}' with 1024-dim vectors")

    def store_embeddings(self, embeddings: List[Dict[str, Any]], batch_size: int = None) -> bool:
        """
        Store embeddings in Qdrant.

        Args:
            embeddings: List of embedding dictionaries with vector and metadata
            batch_size: Number of embeddings to upload in each batch (optional, uses config if not provided)

        Returns:
            True if storage was successful, False otherwise
        """
        try:
            # Load configuration and use provided batch size or fall back to config
            config = get_config()
            actual_batch_size = batch_size if batch_size is not None else config.processing.batch_size

            # Prepare points for insertion
            points = []
            import uuid
            for i, emb in enumerate(embeddings):
                point = models.PointStruct(
                    id=str(uuid.uuid4()),  # Use unique UUID for each point
                    vector=emb['vector'],
                    payload={
                        'source_url': emb.get('source_url', ''),
                        'document_title': emb.get('document_title', ''),
                        'chunk_content': emb['content'],
                        'chunk_id': emb['chunk_id'],
                        'document_id': emb.get('document_id', ''),
                        'chunk_number': emb.get('chunk_number', 0),
                        'model_used': emb.get('model_used', ''),
                        'token_count': emb.get('token_count', 0),
                        'created_at': emb.get('created_at', ''),
                        'metadata': emb.get('metadata', {})
                    }
                )
                points.append(point)

            # Upload points to Qdrant in batches
            for i in range(0, len(points), actual_batch_size):
                batch = points[i:i + actual_batch_size]
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )
                self.logger.info(f"Uploaded batch of {len(batch)} embeddings to Qdrant")

            self.logger.info(f"Successfully stored {len(embeddings)} embeddings in Qdrant collection '{self.collection_name}'")
            return True
        except Exception as e:
            self.logger.error(f"Error storing embeddings in Qdrant: {str(e)}")
            raise

    def search_similar(self, query_vector: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.

        Args:
            query_vector: The query vector to search for similarities
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing similar vectors and their metadata
        """
        try:
            # Get available methods on the client
            available_methods = [method for method in dir(self.client) if not method.startswith('_')]

            search_result = None

            # Method 1: Try the newer query_points method (newest versions)
            if 'query_points' in available_methods:
                try:
                    search_result = self.client.query_points(
                        collection_name=self.collection_name,
                        query=query_vector,
                        limit=limit,
                        with_payload=True,
                        with_vectors=False  # Don't retrieve vectors for performance
                    )
                    self.logger.info("Using query_points method")
                except Exception as e:
                    self.logger.debug(f"query_points failed: {str(e)}")

            # Method 2: Try the query method if it exists
            if search_result is None and 'query' in available_methods:
                try:
                    search_result = self.client.query(
                        collection_name=self.collection_name,
                        query=query_vector,
                        limit=limit
                    )
                    self.logger.info("Using query method")
                except Exception as e:
                    self.logger.debug(f"query method failed: {str(e)}")

            # Method 3: Try the older search method
            if search_result is None and 'search' in available_methods:
                try:
                    search_result = self.client.search(
                        collection_name=self.collection_name,
                        query_vector=query_vector,
                        limit=limit
                    )
                    self.logger.info("Using search method")
                except Exception as e:
                    self.logger.debug(f"search method failed: {str(e)}")

            # Method 4: Try search_points method (older versions)
            if search_result is None and 'search_points' in available_methods:
                try:
                    search_result = self.client.search_points(
                        collection_name=self.collection_name,
                        vector=query_vector,
                        limit=limit,
                        with_payload=True,
                        with_vectors=False
                    )
                    self.logger.info("Using search_points method")
                except Exception as e:
                    self.logger.debug(f"search_points method failed: {str(e)}")

            # Process results if we found any
            if search_result is not None:
                results = []

                # Handle the QueryResponse object that contains points
                # The search_result is a QueryResponse object that has a 'points' attribute
                if hasattr(search_result, 'points'):
                    # This is the correct structure: QueryResponse with points list
                    search_points = search_result.points
                else:
                    # Fallback: assume it's already a list of points
                    search_points = search_result if isinstance(search_result, list) else []

                # Process each point in the results
                for result in search_points:
                    # Handle ScoredPoint format which has id, score, payload attributes
                    result_id = getattr(result, 'id', getattr(result, 'idx', getattr(result, 'pk', None)))
                    result_score = getattr(result, 'score', getattr(result, 'similarity', getattr(result, 'distance', 0)))
                    result_payload = getattr(result, 'payload', getattr(result, 'metadata', {}))
                    result_vector = getattr(result, 'vector', None)

                    results.append({
                        'id': result_id,
                        'score': result_score,
                        'payload': result_payload,
                        'vector': result_vector
                    })

                self.logger.info(f"Found {len(results)} similar vectors in Qdrant")
                return results
            else:
                # If no method worked, return empty results
                self.logger.warning("No working search method found for Qdrant client")
                self.logger.debug(f"Available methods on client: {available_methods}")
                return []

        except Exception as e:
            # If search fails, return empty results but log the error
            self.logger.error(f"Error searching for similar vectors in Qdrant: {str(e)}")
            self.logger.warning("Returning empty results due to search error")
            return []

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection information
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                'collection_name': self.collection_name,
                'vector_size': info.config.params.vectors.size,
                'distance': info.config.params.vectors.distance,
                'point_count': info.points_count,
                'status': info.status
            }
        except Exception as e:
            self.logger.error(f"Error getting collection info: {str(e)}")
            raise

    def delete_collection(self) -> bool:
        """
        Delete the collection (use with caution).

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            self.client.delete_collection(self.collection_name)
            self.logger.info(f"Deleted collection '{self.collection_name}' from Qdrant")
            return True
        except Exception as e:
            self.logger.error(f"Error deleting collection from Qdrant: {str(e)}")
            raise