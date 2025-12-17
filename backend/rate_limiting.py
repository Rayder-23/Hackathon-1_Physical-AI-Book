import time
from typing import Dict, Optional
from collections import defaultdict
from datetime import datetime, timedelta
import threading

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
                return False

    def get_reset_time(self, identifier: str) -> Optional[float]:
        """
        Get the time when the rate limit will reset for the identifier

        Args:
            identifier: Unique identifier for the requester

        Returns:
            float: Unix timestamp when the rate limit will reset, or None if not limited
        """
        with self.lock:
            if identifier in self.requests_log and len(self.requests_log[identifier]) >= self.requests:
                # The reset time is the oldest request time + window
                oldest_request = min(self.requests_log[identifier])
                return oldest_request + self.window
            return None

# Global rate limiter instance
# Default: 100 requests per hour per IP
rate_limiter = InMemoryRateLimiter(
    requests=int(__import__('os').environ.get('RATE_LIMIT_REQUESTS', 100)),
    window=int(__import__('os').environ.get('RATE_LIMIT_WINDOW', 3600))
)