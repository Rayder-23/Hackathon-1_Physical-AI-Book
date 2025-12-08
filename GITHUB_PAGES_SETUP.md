# GitHub Pages Deployment Guide

This guide explains how to properly deploy the Physical AI & Humanoid Robotics Book to GitHub Pages and troubleshoot common issues like the 404 error.

## Prerequisites

1. **GitHub Repository**: Ensure your code is pushed to a GitHub repository
2. **Git Configuration**: Make sure git is properly configured with user name and email:
   ```bash
   git config --global user.name "Your GitHub Username"
   git config --global user.email "your-email@example.com"
   ```

## Deployment Steps

### Option 1: Using Docusaurus Deploy Command

1. **Set Environment Variables** (required for deployment):
   ```bash
   # Windows (PowerShell)
   $env:GIT_USER="your-github-username"; npx docusaurus deploy

   # Windows (Command Prompt)
   set GIT_USER=your-github-username && npx docusaurus deploy

   # Linux/macOS
   export GIT_USER=your-github-username && npx docusaurus deploy
   ```

2. **Alternative with SSH** (if you prefer SSH):
   ```bash
   # Windows (PowerShell)
   $env:GIT_USER="your-github-username"; $env:USE_SSH="true"; npx docusaurus deploy

   # Linux/macOS
   USE_SSH=true GIT_USER=your-github-username npx docusaurus deploy
   ```

### Option 2: Manual Deployment

1. **Build the site**:
   ```bash
   npm run build
   ```

2. **Deploy manually using gh-pages**:
   ```bash
   npx gh-pages -d build -t true
   ```

## GitHub Repository Configuration

After deployment, ensure GitHub Pages is properly configured:

1. Go to your repository on GitHub
2. Navigate to **Settings** → **Pages**
3. Under **Source**, select:
   - **Branch**: `gh-pages`
   - **Folder**: `/ (root)`
4. Click **Save**

## Troubleshooting 404 Errors

If you're seeing a 404 error after deployment, check these common issues:

### 1. Verify Deployment Branch
- Ensure the `gh-pages` branch exists in your repository
- Check that it contains the built site files (index.html, assets, etc.)

### 2. Check Base URL Configuration
In `docusaurus.config.js`, ensure these settings are correct:
```js
baseUrl: '/Hackathon-1_Physical-AI-Book/',  // Should match repo name
organizationName: 'Rayder-23',             // Your GitHub username
projectName: 'Hackathon-1_Physical-AI-Book', // Your repository name
```

### 3. Repository Name Matching
The `projectName` in `docusaurus.config.js` must exactly match your GitHub repository name (case-sensitive).

### 4. Wait for Propagation
After configuring GitHub Pages, it may take 1-10 minutes for the site to become available.

### 5. Check GitHub Actions (if using CI/CD)
If you're using GitHub Actions for deployment, check the workflow logs for any errors.

## Expected Site URL

Once properly deployed, your site will be available at:
```
https://rayder-23.github.io/Hackathon-1_Physical-AI-Book/
```

## Verification Steps

After deployment, verify your site is working:

1. Check that GitHub Pages shows "Your site is published at [URL]"
2. Visit your site and ensure:
   - Homepage loads correctly
   - Navigation works
   - All modules are accessible
   - Personalization features work
   - Translation features work
   - Authentication flows work (if applicable)

## Common Fixes for 404 Errors

### If using a custom domain:
- Ensure your CNAME file is properly configured
- Check DNS settings

### If using the default GitHub Pages URL:
- Verify the repository name matches the baseUrl exactly
- Ensure the `gh-pages` branch has content
- Check that GitHub Pages source is set to the correct branch

### If site shows but assets don't load:
- Verify the `baseUrl` in `docusaurus.config.js` matches your GitHub Pages URL structure
- Check that all asset paths are relative to the baseUrl

## Environment Variables for Deployment

| Variable | Purpose | Example |
|----------|---------|---------|
| `GIT_USER` | GitHub username for commit author | `your-username` |
| `USE_SSH` | Use SSH instead of HTTPS | `true` or `false` |
| `DEPLOYMENT_BRANCH` | Branch to deploy to | `gh-pages` (default) |

## Deployment Script

For convenience, you can use the provided deployment script:

```bash
# Windows (PowerShell)
.\scripts\deploy-to-github-pages.ps1

# Linux/macOS
chmod +x scripts/deploy-to-github-pages.sh
./scripts/deploy-to-github-pages.sh
```

## Additional Resources

- [Docusaurus GitHub Pages Deployment Guide](https://docusaurus.io/docs/deployment#github-pages)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- [Troubleshooting GitHub Pages](https://docs.github.com/en/pages/getting-started-with-github-pages/troubleshooting-custom-domains-and-github-pages#http-404-errors)