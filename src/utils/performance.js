// Performance utilities for Physical AI Book
// Optimizations to ensure page load times under 3 seconds

// Performance monitoring utilities
export const performanceMonitor = {
  // Measure component render time
  measureRender: async (componentName, renderFunction) => {
    if (typeof window === 'undefined') return await renderFunction();

    const start = performance.now();
    const result = await renderFunction();
    const end = performance.now();

    console.debug(`Render time for ${componentName}: ${end - start} ms`);

    // Log slow renders (anything over 16ms is potentially problematic for 60fps)
    if (end - start > 16) {
      console.warn(`Slow render detected in ${componentName}: ${end - start} ms`);
    }

    return result;
  },

  // Lazy load heavy components
  lazyLoadComponent: (importFunction, componentName) => {
    if (typeof window === 'undefined') {
      // On server, return null or a placeholder
      return () => null;
    }

    // Dynamically import component for client-side loading
    return React.lazy(async () => {
      const startTime = performance.now();
      const module = await importFunction();
      const endTime = performance.now();

      console.log(`Lazy load time for ${componentName}: ${endTime - startTime} ms`);

      return module;
    });
  },

  // Debounce utility for performance
  debounce: (func, wait) => {
    let timeout;
    return function executedFunction(...args) {
      const later = () => {
        clearTimeout(timeout);
        func(...args);
      };
      clearTimeout(timeout);
      timeout = setTimeout(later, wait);
    };
  },

  // Throttle utility for performance
  throttle: (func, limit) => {
    let inThrottle;
    return function() {
      const args = arguments;
      const context = this;
      if (!inThrottle) {
        func.apply(context, args);
        inThrottle = true;
        setTimeout(() => inThrottle = false, limit);
      }
    };
  }
};

// Image optimization utilities
export const imageOptimizer = {
  // Generate optimized image URLs
  getOptimizedImageUrl: (src, width = null, quality = 80) => {
    if (typeof window === 'undefined') return src;

    // For static hosting, we can't use dynamic image optimization
    // But we can recommend optimized formats and sizes
    const url = new URL(src, window.location.origin);

    // Add query parameters for CDN-based optimization if available
    if (width) url.searchParams.set('w', width);
    url.searchParams.set('q', quality);

    return url.toString();
  },

  // Preload critical images
  preloadImage: (src) => {
    if (typeof window === 'undefined') return;

    return new Promise((resolve, reject) => {
      const img = new Image();
      img.onload = resolve;
      img.onerror = reject;
      img.src = src;
    });
  },

  // Lazy loading for images
  lazyLoadImage: (imgElement, callback) => {
    if (typeof IntersectionObserver !== 'undefined') {
      const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            const img = entry.target;
            const src = img.dataset.src;

            if (src) {
              img.src = src;
              img.removeAttribute('data-src');

              if (callback) callback(img);
            }

            observer.unobserve(img);
          }
        });
      });

      observer.observe(imgElement);
    } else {
      // Fallback for older browsers
      const img = imgElement;
      const src = img.dataset.src;

      if (src) {
        img.src = src;
        img.removeAttribute('data-src');

        if (callback) callback(img);
      }
    }
  }
};

// Asset optimization utilities
export const assetOptimizer = {
  // Preload critical assets
  preloadCriticalAssets: async (assets) => {
    if (typeof window === 'undefined') return;

    const promises = assets.map(asset => {
      return new Promise((resolve, reject) => {
        const link = document.createElement('link');
        link.rel = 'preload';
        link.as = asset.type || 'fetch'; // script, style, image, font, etc.
        link.href = asset.url;
        link.onload = resolve;
        link.onerror = reject;
        document.head.appendChild(link);
      });
    });

    try {
      await Promise.all(promises);
      console.log('Critical assets preloaded successfully');
    } catch (error) {
      console.error('Error preloading assets:', error);
    }
  },

  // Optimize script loading
  loadScript: (src, options = {}) => {
    return new Promise((resolve, reject) => {
      if (typeof window === 'undefined') {
        resolve();
        return;
      }

      const script = document.createElement('script');
      script.src = src;

      if (options.async) script.async = true;
      if (options.defer) script.defer = true;
      if (options.type) script.type = options.type;

      script.onload = resolve;
      script.onerror = reject;

      document.head.appendChild(script);
    });
  },

  // Optimize CSS loading
  loadCSS: (href) => {
    return new Promise((resolve, reject) => {
      if (typeof window === 'undefined') {
        resolve();
        return;
      }

      const link = document.createElement('link');
      link.rel = 'stylesheet';
      link.href = href;
      link.onload = resolve;
      link.onerror = reject;

      document.head.appendChild(link);
    });
  }
};

// Caching utilities
export const caching = {
  // Cache API wrapper for browser environments
  setCache: async (key, data, ttlMinutes = 60) => {
    if (typeof window === 'undefined' || !window.caches) {
      // Fallback to localStorage for environments without Cache API
      const item = {
        data,
        expiry: new Date().getTime() + (ttlMinutes * 60 * 1000)
      };
      try {
        window.localStorage.setItem(key, JSON.stringify(item));
        return true;
      } catch (e) {
        console.error('Failed to cache in localStorage:', e);
        return false;
      }
    }

    try {
      const cache = await window.caches.open('physical-ai-book-cache');
      const response = new Response(JSON.stringify(data));
      await cache.put(key, response);
      return true;
    } catch (e) {
      console.error('Failed to cache:', e);
      return false;
    }
  },

  getCache: async (key) => {
    if (typeof window === 'undefined') return null;

    // Check localStorage first (for environments without Cache API)
    try {
      const cached = window.localStorage.getItem(key);
      if (cached) {
        const item = JSON.parse(cached);
        if (item.expiry && item.expiry > new Date().getTime()) {
          return item.data;
        } else {
          // Expired, remove from cache
          window.localStorage.removeItem(key);
          return null;
        }
      }
    } catch (e) {
      console.error('Failed to get from localStorage cache:', e);
    }

    // Try Cache API
    if (window.caches) {
      try {
        const cache = await window.caches.open('physical-ai-book-cache');
        const response = await cache.match(key);
        if (response) {
          return await response.json();
        }
      } catch (e) {
        console.error('Failed to get from Cache API:', e);
      }
    }

    return null;
  },

  // Clear expired cache entries from localStorage
  clearExpiredCache: () => {
    if (typeof window === 'undefined' || !window.localStorage) return;

    const now = new Date().getTime();
    for (let i = 0; i < window.localStorage.length; i++) {
      const key = window.localStorage.key(i);
      if (key && key.startsWith('cache:')) {
        try {
          const item = JSON.parse(window.localStorage.getItem(key));
          if (item.expiry && item.expiry < now) {
            window.localStorage.removeItem(key);
          }
        } catch (e) {
          // Ignore parsing errors
        }
      }
    }
  }
};

// Bundle optimization utilities
export const bundleOptimizer = {
  // Code splitting utilities
  dynamicImport: async (modulePath) => {
    try {
      if (typeof window === 'undefined') {
        // Server-side - return a mock implementation
        return { default: () => null };
      }

      // Use dynamic import with performance monitoring
      const startTime = performance.now();
      const module = await import(modulePath);
      const endTime = performance.now();

      console.log(`Dynamic import time for ${modulePath}: ${endTime - startTime} ms`);
      return module;
    } catch (error) {
      console.error(`Failed to dynamically import ${modulePath}:`, error);
      throw error;
    }
  },

  // Chunk size analyzer
  analyzeChunkSizes: () => {
    if (typeof window === 'undefined') return;

    // Analyze webpack chunks if available
    if (window.__chunkMapping) {
      console.group('Chunk Size Analysis');
      Object.entries(window.__chunkMapping).forEach(([chunk, size]) => {
        const sizeKB = (size / 1024).toFixed(2);
        const status = size > 100000 ? '⚠️ Large' : '✅ OK'; // >100KB is large
        console.log(`${chunk}: ${sizeKB} KB ${status}`);
      });
      console.groupEnd();
    }
  }
};

// Overall performance optimizer
export const optimizePerformance = {
  // Initialize performance optimizations
  init: () => {
    if (typeof window === 'undefined') return;

    console.log('Initializing performance optimizations...');

    // Clear expired cache entries
    caching.clearExpiredCache();

    // Analyze chunk sizes if in development
    if ((typeof process !== 'undefined' && process.env && process.env.NODE_ENV) === 'development') {
      bundleOptimizer.analyzeChunkSizes();
    }

    // Set up performance monitoring
    if ('performance' in window) {
      // Monitor paint timing
      if ('paint' in window.performance) {
        window.performance.getEntriesByType('paint').forEach(entry => {
          console.log(`${entry.name}: ${entry.startTime} ms`);
        });
      }
    }

    console.log('Performance optimizations initialized');
  },

  // Optimize page load
  optimizePageLoad: async (criticalAssets = []) => {
    if (typeof window === 'undefined') return;

    // Preload critical assets
    if (criticalAssets.length > 0) {
      await assetOptimizer.preloadCriticalAssets(criticalAssets);
    }

    // Optimize font loading
    optimizePerformance.optimizeFonts();

    // Optimize images that are above the fold
    optimizePerformance.optimizeAboveFoldImages();
  },

  // Optimize fonts
  optimizeFonts: () => {
    if (typeof window === 'undefined') return;

    // Add font-display: swap to improve loading
    const fontLinks = document.querySelectorAll('link[rel="stylesheet"][href*="font"]');
    fontLinks.forEach(link => {
      // For Google Fonts, we can add display=swap parameter
      if (link.href.includes('fonts.googleapis.com')) {
        const newHref = link.href.replace(/(&|\?)display=\w+/, '$1display=swap').replace('?display=swap&', '?display=swap');
        if (newHref !== link.href) {
          link.href = newHref;
        } else if (!link.href.includes('display=')) {
          link.href = link.href + (link.href.includes('?') ? '&' : '?') + 'display=swap';
        }
      }
    });

    // Add font preloading
    const fontFaces = [...document.fonts].filter(font => font.family);
    fontFaces.forEach(font => {
      const preloadLink = document.createElement('link');
      preloadLink.rel = 'preload';
      preloadLink.as = 'font';
      preloadLink.type = 'font/woff2';
      preloadLink.href = font.family; // This is a simplification
      preloadLink.crossOrigin = 'anonymous';
      document.head.appendChild(preloadLink);
    });
  },

  // Optimize images above the fold
  optimizeAboveFoldImages: () => {
    if (typeof window === 'undefined') return;

    // Find images in viewport initially
    const aboveFoldImages = Array.from(document.images).filter(img => {
      const rect = img.getBoundingClientRect();
      return rect.top < window.innerHeight && rect.bottom > 0;
    });

    // Ensure these images are loaded with high priority
    aboveFoldImages.forEach(img => {
      if (img.loading === 'lazy') {
        img.loading = 'eager';
      }
    });
  }
};

// Initialize performance optimizations when DOM is ready
if (typeof window !== 'undefined' && typeof document !== 'undefined') {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      optimizePerformance.init();
    });
  } else {
    // DOM is already ready
    optimizePerformance.init();
  }
}

// Performance metrics tracker
export const performanceMetrics = {
  // Track Largest Contentful Paint (LCP)
  trackLCP: (callback) => {
    if (typeof window === 'undefined' || !('PerformanceObserver' in window)) return;

    new PerformanceObserver((entryList) => {
      const entries = entryList.getEntries();
      const lastEntry = entries[entries.length - 1];
      if (callback) callback(lastEntry);
    }).observe({ entryTypes: ['largest-contentful-paint'] });
  },

  // Track Cumulative Layout Shift (CLS)
  trackCLS: (callback) => {
    if (typeof window === 'undefined' || !('PerformanceObserver' in window)) return;

    let clsValue = 0;
    new PerformanceObserver((entryList) => {
      for (const entry of entryList.getEntries()) {
        if (!entry.hadRecentInput) {
          clsValue += entry.value;
        }
      }
      if (callback) callback(clsValue);
    }).observe({ entryTypes: ['layout-shift'] });
  },

  // Track First Input Delay (FID)
  trackFID: (callback) => {
    if (typeof window === 'undefined' || !('PerformanceObserver' in window)) return;

    new PerformanceObserver((entryList) => {
      for (const entry of entryList.getEntries()) {
        if (callback) callback(entry.processingStart - entry.startTime);
      }
    }).observe({ entryTypes: ['first-input'] });
  }
};