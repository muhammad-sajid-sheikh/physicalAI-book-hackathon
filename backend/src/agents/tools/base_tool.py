"""Base tool class for the AI agent tools."""

from abc import ABC, abstractmethod
from typing import Any, Dict, List
import logging


class BaseTool(ABC):
    """
    Abstract base class for all agent tools.
    Defines the interface that all tools must implement.
    """

    def __init__(self, name: str, description: str):
        """
        Initialize the base tool.

        Args:
            name: Name of the tool
            description: Description of what the tool does
        """
        self.name = name
        self.description = description
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    @abstractmethod
    def execute(self, **kwargs) -> Dict[str, Any]:
        """
        Execute the tool with the provided arguments.

        Args:
            **kwargs: Tool-specific arguments

        Returns:
            Dictionary containing the result of the tool execution
        """
        pass

    def validate_inputs(self, **kwargs) -> bool:
        """
        Validate the inputs to the tool.

        Args:
            **kwargs: Tool-specific arguments

        Returns:
            True if inputs are valid, False otherwise
        """
        # Default implementation - subclasses can override
        return True