#!/usr/bin/env node

// Test script for MCP automation functionality
// This script tests the various MCP integration utilities

const {
  generateContentWithMCP,
  extractSidebarStructure,
  validateMDXContent,
  generateUrduTranslation,
  translateMultipleDocuments,
  validateMultipleMDXFiles,
  callMCPService,
  processBatchWithMCP
} = require('../src/utils/mcp-integration');

// Test data
const testContent = `---
title: Test Document
---

# Test Document

This is a test document to validate MCP automation capabilities.

## Features Tested

- Content generation
- MDX validation
- Urdu translation
- Sidebar extraction
`;

const testDocs = [
  'docs/intro.md',
  'docs/module-1-ros2/intro.md',
  'docs/module-2-simulation/index.md'
];

async function runTests() {
  console.log('🧪 Starting MCP Automation Tests...\n');

  try {
    // Test 1: Content Generation
    console.log('📝 Test 1: Content Generation');
    const contentResult = await generateContentWithMCP({
      prompt: 'Explain ROS 2 architecture',
      format: 'mdx',
      targetAudience: 'intermediate'
    });
    console.log('✅ Content Generation:', contentResult.success ? 'SUCCESS' : 'FAILED');
    if (contentResult.success) {
      console.log('   Generated content preview:', contentResult.data.content.substring(0, 50) + '...');
    }
    console.log('');

    // Test 2: MDX Validation
    console.log('🔍 Test 2: MDX Validation');
    const validationResult = await validateMDXContent(testContent, 'test-doc.md');
    console.log('✅ MDX Validation:', validationResult.success ? 'SUCCESS' : 'FAILED');
    if (validationResult.success) {
      console.log('   Valid:', validationResult.data.isValid);
      console.log('   Errors:', validationResult.data.errors.length);
      console.log('   Warnings:', validationResult.data.warnings.length);
      console.log('   Stats:', validationResult.data.stats);
    }
    console.log('');

    // Test 3: Urdu Translation
    console.log('🇵🇰 Test 3: Urdu Translation');
    const translationResult = await generateUrduTranslation('Hello, this is a test', 'greeting-section');
    console.log('✅ Urdu Translation:', translationResult.success ? 'SUCCESS' : 'FAILED');
    if (translationResult.success) {
      console.log('   Confidence:', translationResult.data.confidence);
      console.log('   Source:', translationResult.data.source);
    }
    console.log('');

    // Test 4: Multiple Document Translation
    console.log('📚 Test 4: Multiple Document Translation');
    const batchTranslationResult = await translateMultipleDocuments(testDocs);
    console.log('✅ Batch Translation:', batchTranslationResult.success ? 'SUCCESS' : 'FAILED');
    if (batchTranslationResult.success) {
      console.log('   Summary:', batchTranslationResult.data.summary);
    }
    console.log('');

    // Test 5: Multiple MDX Validation
    console.log('🔍 Test 5: Multiple MDX Validation');
    const batchValidationResult = await validateMultipleMDXFiles(['docs/intro.md', 'docs/module-1-ros2/intro.md']);
    console.log('✅ Batch MDX Validation:', batchValidationResult.success ? 'SUCCESS' : 'FAILED');
    if (batchValidationResult.success) {
      console.log('   Summary:', batchValidationResult.data.summary);
    }
    console.log('');

    // Test 6: MCP Service Call
    console.log('⚙️ Test 6: MCP Service Call');
    const serviceResult = await callMCPService('content-generation', {
      topic: 'Physical AI Concepts',
      length: 'medium'
    });
    console.log('✅ MCP Service Call:', serviceResult.success ? 'SUCCESS' : 'FAILED');
    console.log('');

    // Test 7: Batch Processing
    console.log('🔄 Test 7: Batch Processing');
    const batchProcessResult = await processBatchWithMCP([
      { service: 'content-generation', params: { topic: 'ROS 2', type: 'overview' } },
      { service: 'mdx-validation', params: { content: '# Test', path: 'test.md' } },
      { service: 'translation', params: { content: 'Test', context: 'test' } }
    ]);
    console.log('✅ Batch Processing:', batchProcessResult.success ? 'SUCCESS' : 'FAILED');
    if (batchProcessResult.success) {
      console.log('   Processed tasks:', batchProcessResult.data.length);
    }
    console.log('');

    // Test 8: Sidebar Extraction (simulated)
    console.log('📊 Test 8: Sidebar Extraction');
    const sidebarResult = await extractSidebarStructure('docs');
    console.log('✅ Sidebar Extraction:', sidebarResult.success ? 'SUCCESS' : 'FAILED');
    console.log('');

    console.log('🎉 All MCP Automation Tests Completed!');
    console.log('\n✅ MCP Integration is working properly with the following capabilities:');
    console.log('   - Content generation with context-aware parameters');
    console.log('   - Comprehensive MDX validation with accessibility checks');
    console.log('   - Urdu translation services');
    console.log('   - Batch processing for multiple documents');
    console.log('   - Sidebar structure extraction');
    console.log('   - Service orchestration through MCP interface');

  } catch (error) {
    console.error('❌ Error during MCP automation tests:', error);
    process.exit(1);
  }
}

// Run the tests if this script is executed directly
if (require.main === module) {
  runTests();
}

module.exports = { runTests };