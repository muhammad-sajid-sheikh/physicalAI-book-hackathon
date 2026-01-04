"""Logging configuration for the Docusaurus Content Ingestion Pipeline."""

import logging
import sys
from typing import Optional
from datetime import datetime
import os


def setup_logging(level: str = "INFO", log_file: Optional[str] = None) -> logging.Logger:
    """
    Set up logging configuration for the application.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path to write logs to

    Returns:
        Configured logger instance
    """
    # Create logger
    logger = logging.getLogger('docusaurus_pipeline')

    # Set level
    numeric_level = getattr(logging, level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f'Invalid log level: {level}')
    logger.setLevel(numeric_level)

    # Clear existing handlers to avoid duplicates
    logger.handlers.clear()

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
    )

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # Create file handler if specified
    if log_file:
        # Ensure log directory exists
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    # Prevent propagation to root logger to avoid duplicate logs
    logger.propagate = False

    return logger


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name, configured with the same settings.

    Args:
        name: Name for the logger

    Returns:
        Configured logger instance
    """
    # This will inherit the configuration from the root logger
    return logging.getLogger(name)


class PipelineLogger:
    """
    A wrapper class for consistent logging across the pipeline.
    """

    def __init__(self, name: str = "docusaurus_pipeline"):
        self.logger = logging.getLogger(name)

    def info(self, message: str, extra: dict = None):
        """Log an info message."""
        if extra:
            message = f"{message} | Extra: {extra}"
        self.logger.info(message)

    def warning(self, message: str, extra: dict = None):
        """Log a warning message."""
        if extra:
            message = f"{message} | Extra: {extra}"
        self.logger.warning(message)

    def error(self, message: str, extra: dict = None, exc_info: bool = False):
        """Log an error message."""
        if extra:
            message = f"{message} | Extra: {extra}"
        self.logger.error(message, exc_info=exc_info)

    def debug(self, message: str, extra: dict = None):
        """Log a debug message."""
        if extra:
            message = f"{message} | Extra: {extra}"
        self.logger.debug(message)

    def critical(self, message: str, extra: dict = None):
        """Log a critical message."""
        if extra:
            message = f"{message} | Extra: {extra}"
        self.logger.critical(message)


def log_execution_time(func):
    """
    Decorator to log the execution time of functions.
    """
    def wrapper(*args, **kwargs):
        start_time = datetime.now()
        try:
            result = func(*args, **kwargs)
            end_time = datetime.now()
            execution_time = (end_time - start_time).total_seconds()
            logger = logging.getLogger(func.__module__)
            logger.info(f"{func.__name__} executed in {execution_time:.2f} seconds")
            return result
        except Exception as e:
            end_time = datetime.now()
            execution_time = (end_time - start_time).total_seconds()
            logger = logging.getLogger(func.__module__)
            logger.error(f"{func.__name__} failed after {execution_time:.2f} seconds: {str(e)}")
            raise
    return wrapper


def log_retry_attempt(func):
    """
    Decorator to log retry attempts for functions.
    """
    def wrapper(*args, **kwargs):
        max_retries = kwargs.get('max_retries', 3)
        for attempt in range(max_retries + 1):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                if attempt == max_retries:
                    logger = logging.getLogger(func.__module__)
                    logger.error(f"{func.__name__} failed after {max_retries} retry attempts: {str(e)}", exc_info=True)
                    raise
                else:
                    logger = logging.getLogger(func.__module__)
                    logger.warning(f"{func.__name__} attempt {attempt + 1} failed: {str(e)}, retrying...")
    return wrapper