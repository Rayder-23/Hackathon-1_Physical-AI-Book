// Verification script for GitHub Pages configuration
// Checks if the Docusaurus configuration is properly set up for GitHub Pages deployment

const fs = require('fs');
const path = require('path');

function verifyGitHubPagesConfig() {
  console.log('🔍 Verifying GitHub Pages Configuration...\n');

  // Check docusaurus.config.js exists
  const configPath = path.join(__dirname, '../docusaurus.config.js');
  if (!fs.existsSync(configPath)) {
    console.error('❌ docusaurus.config.js not found');
    return false;
  }

  // Read the configuration file
  const configFile = fs.readFileSync(configPath, 'utf8');

  // Parse key configuration values
  const baseUrlMatch = configFile.match(/baseUrl:\s*['"]([^'"]+)['"]/);
  const organizationNameMatch = configFile.match(/organizationName:\s*['"]([^'"]+)['"]/);
  const projectNameMatch = configFile.match(/projectName:\s*['"]([^'"]+)['"]/);
  const urlMatch = configFile.match(/url:\s*['"]([^'"]+)['"]/);

  const baseUrl = baseUrlMatch ? baseUrlMatch[1] : null;
  const organizationName = organizationNameMatch ? organizationNameMatch[1] : null;
  const projectName = projectNameMatch ? projectNameMatch[1] : null;
  const url = urlMatch ? urlMatch[1] : null;

  console.log('📋 Configuration Values:');
  console.log(`   baseUrl: ${baseUrl || '❌ NOT FOUND'}`);
  console.log(`   organizationName: ${organizationName || '❌ NOT FOUND'}`);
  console.log(`   projectName: ${projectName || '❌ NOT FOUND'}`);
  console.log(`   url: ${url || '❌ NOT FOUND'}`);
  console.log('');

  // Verify values
  let isValid = true;

  if (!baseUrl) {
    console.error('❌ baseUrl is not configured');
    isValid = false;
  } else {
    console.log(`✅ baseUrl configured: ${baseUrl}`);
  }

  if (!organizationName) {
    console.error('❌ organizationName is not configured');
    isValid = false;
  } else {
    console.log(`✅ organizationName configured: ${organizationName}`);
  }

  if (!projectName) {
    console.error('❌ projectName is not configured');
    isValid = false;
  } else {
    console.log(`✅ projectName configured: ${projectName}`);
  }

  if (!url) {
    console.error('❌ url is not configured');
    isValid = false;
  } else {
    console.log(`✅ url configured: ${url}`);
  }

  // Check if baseUrl is appropriate for GitHub Pages
  if (baseUrl && !baseUrl.startsWith('/')) {
    console.error('❌ baseUrl should start with "/" for GitHub Pages');
    isValid = false;
  } else if (baseUrl) {
    console.log('✅ baseUrl format is correct for GitHub Pages');
  }

  // Check if projectName matches typical GitHub Pages repo naming pattern
  if (projectName && organizationName) {
    const expectedUrl = `https://${organizationName}.github.io/${projectName}/`;
    console.log(`\n🌐 Expected GitHub Pages URL: ${expectedUrl}`);
  }

  // Check if deployment branch is configured
  const deploymentBranchMatch = configFile.match(/deploymentBranch:\s*['"]([^'"]+)['"]/);
  const deploymentBranch = deploymentBranchMatch ? deploymentBranchMatch[1] : 'gh-pages';

  if (deploymentBranch) {
    console.log(`✅ deploymentBranch configured: ${deploymentBranch}`);
  } else {
    console.log('⚠️  deploymentBranch not explicitly configured (using default: gh-pages)');
  }

  // Check if trailingSlash is configured appropriately
  const trailingSlashMatch = configFile.match(/trailingSlash:\s*(true|false)/);
  const trailingSlash = trailingSlashMatch ? trailingSlashMatch[1] : 'not configured';

  if (trailingSlash !== 'not configured') {
    console.log(`✅ trailingSlash configured: ${trailingSlash}`);
  } else {
    console.log('⚠️  trailingSlash not configured (Docusaurus will use default)');
  }

  console.log('\n---');
  if (isValid) {
    console.log('✅ GitHub Pages configuration appears valid!');
    console.log('\nNext steps:');
    console.log('1. Ensure the gh-pages branch exists in your GitHub repository');
    console.log('2. Run: npm run build');
    console.log('3. Set GIT_USER environment variable: $env:GIT_USER="your-username" (PowerShell) or export GIT_USER="your-username" (bash)');
    console.log('4. Run: npx docusaurus deploy');
    console.log('5. In GitHub repo Settings → Pages, set source to gh-pages branch');
  } else {
    console.log('❌ Issues found in GitHub Pages configuration. Please fix before deploying.');
  }

  return isValid;
}

// Run verification if executed directly
if (require.main === module) {
  verifyGitHubPagesConfig();
}

module.exports = { verifyGitHubPagesConfig };