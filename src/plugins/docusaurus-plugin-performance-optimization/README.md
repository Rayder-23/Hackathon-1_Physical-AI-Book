# Docusaurus Performance Optimization Plugin

This plugin implements performance optimizations to ensure page load times remain under 3 seconds for the Physical AI Book.

## Features

- **Bundle Splitting**: Optimizes code splitting to reduce initial bundle size
- **Resource Hints**: Adds preconnect, dns-prefetch, and preload hints for critical resources
- **Font Optimization**: Implements font-display: swap for faster text rendering
- **Performance Monitoring**: Tracks page load times and reports metrics
- **Critical CSS**: Inlines critical CSS for above-the-fold content
- **Async Loading**: Defers non-critical JavaScript loading

## Optimization Goals

- Page load times under 3 seconds
- Largest Contentful Paint (LCP) under 2.5 seconds
- Cumulative Layout Shift (CLS) under 0.1
- First Input Delay (FID) under 100ms

## Implementation Details

### Bundle Optimization
- Vendor code is separated into its own chunk
- Common components are shared across pages
- Maximum chunk size limited to 244KB
- Module concatenation enabled for smaller bundles

### Resource Optimization
- Preconnects to critical third-party origins (Google Fonts, CDNs)
- Preloads critical font files with display swap
- DNS prefetch for CDN resources
- Critical CSS inlined to prevent render-blocking

### Performance Monitoring
- Tracks page load times and logs if exceeding 3 seconds
- Integrates with Google Analytics for performance metrics
- Provides build-time performance summaries

## Usage

The plugin is automatically included in the Docusaurus configuration and applies optimizations during the build process.