import time
from typing import Dict, Optional
from collections import defaultdict
from datetime import datetime, timedelta
import threading
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class InMemoryRateLimiter:
    """
    Simple in-memory rate limiter to prevent API abuse
    """
    def __init__(self, requests: int = 100, window: int = 3600):  # 100 requests per hour by default
        self.requests = requests
        self.window = window  # in seconds
        self.requests_log: Dict[str, list] = defaultdict(list)  # IP -> list of request timestamps
        self.lock = threading.Lock()  # Thread safety for concurrent requests

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed

        Args:
            identifier: Unique identifier for the requester (e.g., IP address)

        Returns:
            bool: True if request is allowed, False if rate limit exceeded
        """
        try:
            with self.lock:
                now = time.time()
                # Remove requests older than the window
                self.requests_log[identifier] = [
                    timestamp for timestamp in self.requests_log[identifier]
                    if now - timestamp < self.window
                ]

                # Check if we're under the limit
                if len(self.requests_log[identifier]) < self.requests:
                    # Add current request
                    self.requests_log[identifier].append(now)
                    return True
                else:
                    logger.warning(f"Rate limit exceeded for identifier: {identifier}")
                    return False
        except Exception as e:
            logger.error(f"Error in rate limiter is_allowed: {str(e)}")
            # In case of error, allow the request to prevent blocking users
            return True

    def get_reset_time(self, identifier: str) -> Optional[float]:
        """
        Get the time when the rate limit will reset for the identifier

        Args:
            identifier: Unique identifier for the requester

        Returns:
            float: Unix timestamp when the rate limit will reset, or None if not limited
        """
        try:
            with self.lock:
                if identifier in self.requests_log and len(self.requests_log[identifier]) >= self.requests:
                    # The reset time is the oldest request time + window
                    oldest_request = min(self.requests_log[identifier])
                    return oldest_request + self.window
                return None
        except Exception as e:
            logger.error(f"Error in rate limiter get_reset_time: {str(e)}")
            return None

    def get_current_usage(self, identifier: str) -> Dict[str, int]:
        """
        Get current usage statistics for an identifier

        Args:
            identifier: Unique identifier for the requester

        Returns:
            dict: Contains current_requests and limit
        """
        try:
            with self.lock:
                now = time.time()
                # Remove requests older than the window
                self.requests_log[identifier] = [
                    timestamp for timestamp in self.requests_log[identifier]
                    if now - timestamp < self.window
                ]
                current_requests = len(self.requests_log[identifier])
                return {
                    "current_requests": current_requests,
                    "limit": self.requests,
                    "remaining": max(0, self.requests - current_requests),
                    "reset_time": self.get_reset_time(identifier)
                }
        except Exception as e:
            logger.error(f"Error in rate limiter get_current_usage: {str(e)}")
            return {
                "current_requests": 0,
                "limit": self.requests,
                "remaining": self.requests,
                "reset_time": None
            }

# Global rate limiter instance
# Default: 100 requests per hour per IP
rate_limiter = InMemoryRateLimiter(
    requests=int(__import__('os').environ.get('RATE_LIMIT_REQUESTS', 100)),
    window=int(__import__('os').environ.get('RATE_LIMIT_WINDOW', 3600))
)