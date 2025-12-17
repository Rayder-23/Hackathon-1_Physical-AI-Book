// MCP Integration Utilities for Physical AI Book
// Provides utilities for automated content generation, sidebar extraction, and MDX validation

// Utility to call MCP server for content generation
export const generateContentWithMCP = async (params) => {
  try {
    // This would typically make a call to an MCP server
    // For static hosting, we'll simulate the functionality
    console.log('MCP Content Generation Request:', params);

    // Simulated response - in real implementation, this would come from MCP server
    return {
      success: true,
      data: {
        content: params.prompt || 'Generated content based on the provided parameters',
        metadata: {
          generatedAt: new Date().toISOString(),
          source: 'MCP Server',
          ...params
        }
      }
    };
  } catch (error) {
    console.error('MCP Content Generation Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Utility to extract sidebar structure from content
export const extractSidebarStructure = async (contentDirectory) => {
  try {
    console.log('MCP Sidebar Extraction Request:', contentDirectory);

    // In a real implementation, this would scan the content directory
    // and generate sidebar structure automatically
    return {
      success: true,
      data: {
        sidebar: {
          type: 'category',
          label: 'Auto-generated Sidebar',
          items: []
        }
      }
    };
  } catch (error) {
    console.error('MCP Sidebar Extraction Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Utility for MDX validation
export const validateMDXContent = async (mdxContent, filePath) => {
  try {
    console.log('MCP MDX Validation Request:', filePath);

    // Perform various validation checks
    const validationResults = {
      isValid: true,
      warnings: [],
      errors: [],
      suggestions: [],
      stats: {
        wordCount: 0,
        imageCount: 0,
        linkCount: 0,
        headingCount: 0
      }
    };

    // Check for proper MDX syntax
    const syntaxIssues = checkMDXSyntax(mdxContent);
    validationResults.errors.push(...syntaxIssues.errors);
    validationResults.warnings.push(...syntaxIssues.warnings);

    // Check for accessibility issues
    const accessibilityIssues = checkAccessibility(mdxContent);
    validationResults.warnings.push(...accessibilityIssues);

    // Check for proper component usage
    const componentIssues = checkComponentUsage(mdxContent);
    validationResults.errors.push(...componentIssues.errors);
    validationResults.warnings.push(...componentIssues.warnings);

    // Check for content structure
    const structureIssues = checkContentStructure(mdxContent);
    validationResults.warnings.push(...structureIssues);

    // Update stats
    validationResults.stats = calculateContentStats(mdxContent);

    // Determine if content is valid based on errors
    validationResults.isValid = validationResults.errors.length === 0;

    return {
      success: true,
      data: validationResults
    };
  } catch (error) {
    console.error('MCP MDX Validation Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Enhanced utility to validate multiple MDX files
export const validateMultipleMDXFiles = async (filePaths) => {
  try {
    console.log('MCP Batch MDX Validation Request:', filePaths.length, 'files');

    const results = [];
    let totalErrors = 0;
    let totalWarnings = 0;

    for (const filePath of filePaths) {
      try {
        const content = require('fs').readFileSync(filePath, 'utf8');
        const validation = await validateMDXContent(content, filePath);
        results.push({
          filePath,
          validation
        });

        if (validation.success) {
          totalErrors += validation.data.errors.length;
          totalWarnings += validation.data.warnings.length;
        }
      } catch (error) {
        results.push({
          filePath,
          validation: {
            success: false,
            error: error.message
          }
        });
      }
    }

    return {
      success: true,
      data: {
        results,
        summary: {
          totalFiles: filePaths.length,
          validFiles: results.filter(r => r.validation.success && r.validation.data.isValid).length,
          invalidFiles: results.filter(r => r.validation.success && !r.validation.data.isValid).length,
          totalErrors,
          totalWarnings
        }
      }
    };
  } catch (error) {
    console.error('MCP Batch MDX Validation Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Utility to fix common MDX issues
export const fixMDXContent = async (mdxContent, filePath, fixOptions = {}) => {
  try {
    console.log('MCP MDX Fix Request:', filePath);

    let fixedContent = mdxContent;

    // Apply various fixes based on options
    if (fixOptions.fixHeadings) {
      fixedContent = fixHeadingHierarchy(fixedContent);
    }

    if (fixOptions.fixLinks) {
      fixedContent = fixBrokenLinks(fixedContent);
    }

    if (fixOptions.addAltText) {
      fixedContent = addMissingAltText(fixedContent);
    }

    return {
      success: true,
      data: {
        originalContent: mdxContent,
        fixedContent,
        changesApplied: [] // Would contain details of changes made
      }
    };
  } catch (error) {
    console.error('MCP MDX Fix Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Helper function to check MDX syntax
function checkMDXSyntax(content) {
  const errors = [];
  const warnings = [];

  // Check for common syntax issues
  // Check for unclosed JSX tags
  const jsxTagMatch = content.match(/<([a-zA-Z][a-zA-Z0-9]*)[^>]*?(?!\/)>/g);
  if (jsxTagMatch) {
    for (const tag of jsxTagMatch) {
      const tagName = tag.match(/<([a-zA-Z][a-zA-Z0-9]*)/)[1];
      const openCount = (content.match(new RegExp(`<${tagName}[^>]*?(?!/)\\s*>`, 'g')) || []).length;
      const closeCount = (content.match(new RegExp(`</${tagName}\\s*>`, 'g')) || []).length;

      if (openCount > closeCount) {
        errors.push(`Unclosed JSX tag: ${tagName}`);
      }
    }
  }

  // Check for proper import statements
  const importRegex = /^import\s+.*\s+from\s+['"].*['"];?$/gm;
  const imports = content.match(importRegex) || [];
  for (const imp of imports) {
    if (!imp.endsWith(';')) {
      warnings.push(`Missing semicolon in import: ${imp.substring(0, 30)}...`);
    }
  }

  return { errors, warnings };
}

// Helper function to check accessibility
function checkAccessibility(content) {
  const warnings = [];

  // Check for missing alt text in images
  const imgRegex = /!\[([^\]]*)\]\([^)]*\)/g;
  let match;
  while ((match = imgRegex.exec(content)) !== null) {
    if (!match[1] || match[1].trim() === '') {
      warnings.push(`Image missing alt text at position ${match.index}`);
    }
  }

  // Check for proper heading hierarchy
  const headingRegex = /^(#{1,6})\s+(.*)$/gm;
  let prevLevel = 0;
  while ((match = headingRegex.exec(content)) !== null) {
    const level = match[1].length;
    if (level > prevLevel + 1) {
      warnings.push(`Improper heading hierarchy: H${prevLevel} followed by H${level} at position ${match.index}`);
    }
    prevLevel = level;
  }

  return warnings;
}

// Helper function to check component usage
function checkComponentUsage(content) {
  const errors = [];
  const warnings = [];

  // Check for properly imported components
  const componentRegex = /<([A-Z][a-zA-Z0-9]*)/g;
  let match;
  while ((match = componentRegex.exec(content)) !== null) {
    const componentName = match[1];
    // In a real implementation, we'd check if the component is properly imported
    // For now, we'll just add a warning for common issues
    if (componentName.length > 20) {
      warnings.push(`Potentially long component name: ${componentName}`);
    }
  }

  return { errors, warnings };
}

// Helper function to check content structure
function checkContentStructure(content) {
  const warnings = [];

  // Check if content has a title (either in frontmatter or as H1)
  const hasFrontmatterTitle = /title:\s*['"][^'"]+['"]/.test(content);
  const hasH1 = /^#\s+.+/m.test(content);

  if (!hasFrontmatterTitle && !hasH1) {
    warnings.push('Content should have a title (either in frontmatter or as H1)');
  }

  // Check for minimum content length
  const textContent = content.replace(/[#\*\-\[\]\(\)`~_]/g, '').trim();
  if (textContent.length < 50) {
    warnings.push('Content appears to be very short');
  }

  return warnings;
}

// Helper function to calculate content stats
function calculateContentStats(content) {
  return {
    wordCount: content.trim().split(/\s+/).length,
    imageCount: (content.match(/!\[([^\]]*)\]\([^)]*\)/g) || []).length,
    linkCount: (content.match(/\[([^\]]+)\]\([^)]*\)/g) || []).length,
    headingCount: (content.match(/^#+\s+.+$/gm) || []).length
  };
}

// Helper function to fix heading hierarchy
function fixHeadingHierarchy(content) {
  // This would implement logic to fix heading levels
  return content; // Placeholder
}

// Helper function to fix broken links
function fixBrokenLinks(content) {
  // This would implement logic to fix or flag broken links
  return content; // Placeholder
}

// Helper function to add missing alt text
function addMissingAltText(content) {
  // This would implement logic to add placeholder alt text
  return content; // Placeholder
}

// Utility to generate Urdu translations
export const generateUrduTranslation = async (englishContent, context) => {
  try {
    console.log('MCP Urdu Translation Request for:', context);

    // In a real implementation, this would generate Urdu translations
    // using an MCP server with translation capabilities
    // For static hosting, we'll simulate the translation
    const urduContent = await simulateUrduTranslation(englishContent);

    return {
      success: true,
      data: {
        urduContent,
        confidence: 0.85,
        source: 'MCP Urdu Translation Service'
      }
    };
  } catch (error) {
    console.error('MCP Urdu Translation Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Enhanced utility to process entire document for Urdu translation
export const translateDocumentToUrdu = async (docPath, options = {}) => {
  try {
    console.log('MCP Document Translation Request:', docPath);

    // In a real implementation, this would read the document, translate it,
    // and handle any special formatting or components
    const { preserveFrontmatter = true, translateHeadings = true } = options;

    // Simulated translation of a document
    return {
      success: true,
      data: {
        translatedPath: docPath.replace('/docs/', '/static/urdu/').replace('.md', '_ur.md'),
        stats: {
          originalWords: 100,
          translatedWords: 100,
          confidence: 0.85
        }
      }
    };
  } catch (error) {
    console.error('MCP Document Translation Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Batch utility to translate multiple documents
export const translateMultipleDocuments = async (docPaths, options = {}) => {
  try {
    console.log('MCP Batch Translation Request for:', docPaths.length, 'documents');

    const results = [];
    for (const docPath of docPaths) {
      const result = await translateDocumentToUrdu(docPath, options);
      results.push({
        path: docPath,
        result
      });
    }

    return {
      success: true,
      data: results,
      summary: {
        total: docPaths.length,
        successful: results.filter(r => r.result.success).length,
        failed: results.filter(r => !r.result.success).length
      }
    };
  } catch (error) {
    console.error('MCP Batch Translation Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Simulate Urdu translation (in a real implementation, this would call an actual translation API)
async function simulateUrduTranslation(englishText) {
  // This is a placeholder - in reality, this would call an MCP translation service
  // For now, we'll just return a simulated translation with Urdu script indicators
  return `<!-- Generated Urdu Translation -->\n${englishText}\n\n<!-- Original: ${englishText.substring(0, 50)}... -->`;
}

// Initialize MCP integration
export const initializeMCPIntegration = async () => {
  try {
    console.log('Initializing MCP Integration...');

    // Setup any required configuration for MCP integration
    // This might include API keys, server endpoints, etc.

    return {
      success: true,
      message: 'MCP Integration initialized successfully'
    };
  } catch (error) {
    console.error('MCP Integration Initialization Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Generic MCP service call
export const callMCPService = async (serviceName, params) => {
  try {
    console.log(`Calling MCP Service: ${serviceName}`, params);

    // This would be the main interface to communicate with MCP servers
    // In static hosting, we'll simulate the behavior
    const serviceMap = {
      'content-generation': generateContentWithMCP,
      'sidebar-extraction': extractSidebarStructure,
      'mdx-validation': validateMDXContent,
      'translation': generateUrduTranslation
    };

    const service = serviceMap[serviceName];
    if (service) {
      return await service(params);
    } else {
      throw new Error(`Unknown MCP service: ${serviceName}`);
    }
  } catch (error) {
    console.error(`MCP Service Call Error (${serviceName}):`, error);
    return {
      success: false,
      error: error.message
    };
  }
};

// Batch processing utility
export const processBatchWithMCP = async (tasks) => {
  try {
    console.log('MCP Batch Processing Request:', tasks);

    const results = [];
    for (const task of tasks) {
      const result = await callMCPService(task.service, task.params);
      results.push({
        ...task,
        result
      });
    }

    return {
      success: true,
      data: results
    };
  } catch (error) {
    console.error('MCP Batch Processing Error:', error);
    return {
      success: false,
      error: error.message
    };
  }
};